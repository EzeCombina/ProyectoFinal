// Librerías
#include "dw3000.h"
#include "SPI.h"

extern SPISettings _fastSPI;  // Importa la configuración de velocidad y modo del bus SPI definida en la librería del DW3000

// Conexión del DW3000 a la ESP
#define PIN_RST 27  // Reset
#define PIN_IRQ 34  // Interupción
#define PIN_SS 4    // Slave select

#define RNG_DELAY_MS 20        // Delay entre ciclos completos
#define PER_POLL_DELAY_MS 25    // Delay corto entre poll a cada Anchor

// valores predetermindados para calibrar las antenas de los módulos
#define TX_ANT_DLY 16385 
#define RX_ANT_DLY 16385 

// Posición de los datos dentro del mensaje
#define ALL_MSG_COMMON_LEN 10          // Longitud minima de los mensajes
#define ALL_MSG_SN_IDX 2               // Número de secuencia
#define RESP_MSG_POLL_RX_TS_IDX 10     // Timestamp de cuando el anchor recibe el mensaje
#define RESP_MSG_RESP_TX_TS_IDX 14     // Timestamp de cuando el tag recibe la respuesta

#define POLL_TX_TO_RESP_RX_DLY_UUS 400 // Tiempo que tarda el tag de pasar de transmisor a receptor
#define RESP_RX_TIMEOUT_UUS 9000       // Timeout de espera del tag

//IDs de los Anchors
#define ANCHOR1_ID 1
#define ANCHOR2_ID 2
#define ANCHOR3_ID 3
#define ANCHOR4_ID 4

// Variables de Filtro (EMA)
double distA1_f = -1; // Se configuran en -1 para saber cuando es la primera lectura
double distA2_f = -1;
double distA3_f = -1;
double distA4_f = -1;

// Configuración de radiofrecuencia del DW3000
static dwt_config_t config = {
    5,                   // Canal 5, equivale a 6,5 GHz
    DWT_PLEN_128,        // Longitud del preambulo
    DWT_PAC8,            // Tamaño del PAC
    9, 9,                // Códigos del preambulo para el TX y el RX
    1,                   // Uso de SFD no estándar
    DWT_BR_6M8,          // Taza de transferencia de datos de 6,8 Mbps
    DWT_PHRMODE_STD,     // Modo de cabecera PHY
    DWT_PHRRATE_STD,     // Taza de cabecera PHY
    (129 + 8 - 8),       // SFD Timeout
    DWT_STS_MODE_OFF,    // STS apagado
    DWT_STS_LEN_64,      // Tamaño del STS - No aplica por estar apagado
    DWT_PDOA_M0          // Modo PDOA
};

// Poll prefabricado
static uint8_t tx_poll_msg[] = {0x41,0x88,0,0xCA,0xDE,'W','A','V','E',0xE0,0,0}; // Trama MAC 802.15.4. Contiene control de frame, secuencias, PAN ID, y "WAVE" como identificador
static uint8_t rx_buffer[20];       // Buffer de memoria para guardar la respuesta del Anchor
static uint8_t frame_seq_nb = 0;    // Contador secuencial de mensajes enviados (0 a 255)
static uint32_t status_reg = 0;     // Variable para leer el estado del registro del chip DW3000

double tof, distance;   // Tiempo de vuelo y distancia final
extern dwt_txconfig_t txconfig_options;  // Carga la configuración de potencia de transmisión de la librería

// Función para medir un anchor
void medirAnchorDirect(uint8_t anchor_id) // Se le envía el ID del anchor que se quiere medir
{
  tx_poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb; // Prepara el mensaje con la secuencia actual
  tx_poll_msg[9] = 0xE0 + anchor_id;    // Define el destino (0xE1, 0xE2, 0xE3, 0xE4)

  // Transmisión
  dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0);          // Escribe el mensaje en la memoria RAM del chip
  dwt_writetxfctrl(sizeof(tx_poll_msg), 0, 1);                   // Configura el control de transmisión
  dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);   // Inicia la transmisión inmediatamente y le dice al chip que se quede esperando una respuesta

  // Recepción
  uint32_t t0 = millis();     //Guarda el tiempo actual
  while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))  // Sistema de seguridad: si el chip UWB se cuelga y no responde nada en 50ms, aborta el bucle
  {
    if (millis() - t0 > 50) break; // Timeout de seguridad del loop
  }

  frame_seq_nb++;  //incrementa el numero de secuencia para el próximo mensaje

  if (status_reg & SYS_STATUS_RXFCG_BIT_MASK)   // Revisa si la condición de salida fue que llegó un mensaje correctamente
  { 
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);       // Limpia la bandera de interrupción de RX en el registro para poder recibir futuros mensajes
    uint32_t frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;  // Lee cuántos bytes tiene el mensaje que acaba de llegar
    
    if (frame_len <= sizeof(rx_buffer))    // Se asegura de que el mensaje no sea más grande que nuestra memoria buffer
    {
      dwt_readrxdata(rx_buffer, frame_len, 0);   // Extrae los datos del chip y los mete en nuestro rx_buffer
      uint8_t resp_id = rx_buffer[3];            // Lee el byte 3 para saber qué ID nos respondió

      if (resp_id == anchor_id)    // Revisa si respondió el Anchor al que se le envió el poll
      {
        uint32_t poll_tx_ts = dwt_readtxtimestamplo32();  // Momento exacto en que enviamos la petición
        uint32_t resp_rx_ts = dwt_readrxtimestamplo32();  // Momento exacto en que recibimos esta respuesta

        // Se extraen los tiempos que midió el anchor
        uint32_t poll_rx_ts, resp_tx_ts;
        resp_msg_get_ts(&rx_buffer[RESP_MSG_POLL_RX_TS_IDX], &poll_rx_ts); // Cuándo el Anchor recibió la petición
        resp_msg_get_ts(&rx_buffer[RESP_MSG_RESP_TX_TS_IDX], &resp_tx_ts); // Cuándo el Anchor envió esta respuesta

        float clockOffsetRatio = ((float)dwt_readclockoffset()) / (uint32_t)(1 << 26);   // Corrige las pequeñas diferencias de tiempo entre Tag y Anchor
        int32_t rtd_init = resp_rx_ts - poll_tx_ts; // Tiempo total que le tomó al Tag enviar y recibir
        int32_t rtd_resp = resp_tx_ts - poll_rx_ts; // Tiempo que el Anchor demoró en procesar y responder

        tof = ((rtd_init - rtd_resp * (1 - clockOffsetRatio)) / 2.0) * DWT_TIME_UNITS;  // Cálculo del Tiempo de Vuelo (ToF)
        distance = tof * SPEED_OF_LIGHT;  // Cálculo de la distancia

        // FILTRO EMA
        double alpha = 0.30;  // Significa que se confía un 30% en  la nueva lectura y un 70% en el historial

        // Clasifica y guarda la distancia filtrada dependiendo de qué Anchor estamos midiendo
        // ANCHOR 1
        if (anchor_id == 1) {
            if (distA1_f < 0) distA1_f = distance;
            distA1_f = distA1_f * (1 - alpha) + distance * alpha;
            Serial.print("A1: "); Serial.print(distA1_f, 2); Serial.print(" m\t");
        }
        // ANCHOR 2
        else if (anchor_id == 2) {
            if (distA2_f < 0) distA2_f = distance;
            distA2_f = distA2_f * (1 - alpha) + distance * alpha;
            Serial.print("A2: "); Serial.print(distA2_f, 2); Serial.print(" m\t");
        }
        // ANCHOR 3 (NUEVO)
        else if (anchor_id == 3) {
            if (distA3_f < 0) distA3_f = distance;
            distA3_f = distA3_f * (1 - alpha) + distance * alpha;
            Serial.print("A3: "); Serial.print(distA3_f, 2); Serial.print(" m\t");
        }
        // ANCHOR 4 (NUEVO)
        else if (anchor_id == 4) {
            if (distA4_f < 0) distA4_f = distance;
            distA4_f = distA4_f * (1 - alpha) + distance * alpha;
            Serial.print("A4: "); Serial.print(distA4_f, 2); Serial.println(" m"); 
            // Usamos println en el último para cerrar la línea
        }
      }
      else {
        // Serial.print("ID inesperado: "); Serial.println(resp_id);
      }
    }
  }
  else
  {
    // Si entró aquí, significa que hubo un error (ruido) o Timeout (el Anchor no respondió)
    // Limpia los registros de error para no quedarse trabado
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
    // Opcional: Imprimir error si quieres depurar pérdidas
    Serial.print("!TO A"); Serial.println(anchor_id); // println CAMBIAR
  }
}

// Inicialización
void setup()
{
  UART_init();              // Inicializa pines UART
  Serial.begin(115200);     // Abre el puerto serie para ver lo datos en la PC a 115200 baudios

  _fastSPI = SPISettings(16000000L, MSBFIRST, SPI_MODE0);   // Configura la velocidad del bus SPI a 16 MHz
  
  spiBegin(PIN_IRQ, PIN_RST);   // Inicializa el SPI con los pines de control
  spiSelect(PIN_SS);            // Asigna el pin de selección de esclavo
  delay(2);                     // Pausa de estabilización

  while (!dwt_checkidlerc()) { Serial.println("IDLE FAILED"); while(1); }   // Verifica que el DW3000 esté encendido y en modo reposo (Idle). Si falla, traba el programa (while(1)).
  if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) { Serial.println("INIT FAILED"); while(1); }   // Inicializa los registros base del DW3000

  dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);   // Enciende los LEDs integrados del chip
  if (dwt_configure(&config)) { Serial.println("CONFIG FAILED"); while(1); }    // Carga la configuración de radiofrecuencia que se declaró anteriormente

  dwt_configuretxrf(&txconfig_options); // Configura la potencia del transmisor de radio
  dwt_setrxantennadelay(RX_ANT_DLY);    // Aplica la compensación de retardo de la antena RX
  dwt_settxantennadelay(TX_ANT_DLY);    // Aplica la compensación de retardo de la antena TX

  // Configura tiempos ciegos y timeouts 
  dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
  dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);

  // Activa el LNA (Low Noise Amplifier) para RX y el PA (Power Amplifier) para TX. Mejora el alcance.
  dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);

  Serial.println("TAG listo: Polling 4 Anchors");
}

void loop() //Bucle principal
{
  //Mide un anchor y espera
  medirAnchorDirect(ANCHOR1_ID);
  delay(PER_POLL_DELAY_MS);

  medirAnchorDirect(ANCHOR2_ID);
  delay(PER_POLL_DELAY_MS);

  medirAnchorDirect(ANCHOR3_ID);
  delay(PER_POLL_DELAY_MS);

  medirAnchorDirect(ANCHOR4_ID);
  delay(PER_POLL_DELAY_MS);

  // Serial.println("---"); // Comentado para que salga todo en una línea si hay respuesta
  delay(RNG_DELAY_MS); // Termina el ciclo de los 4 Anchors y hace una pausa global de 20ms antes de reiniciar todo el proceso
}