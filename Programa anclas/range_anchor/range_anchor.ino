// Librerías
#include "dw3000.h"
#include "SPI.h"

extern SPISettings _fastSPI;  // Importa la configuración de velocidad y modo del bus SPI definida en la librería del DW3000

// Conexión del DW3000 a la ESP
#define PIN_RST 27  // Reset
#define PIN_IRQ 34  // Interupción
#define PIN_SS 4    // Slave select

// valores predetermindados para calibrar las antenas de los módulos
#define TX_ANT_DLY 16385 
#define RX_ANT_DLY 16385 

#define POLL_RX_TO_RESP_TX_DLY_UUS 600   // Tiempo base de retardo entre que recibe el Poll y envía la respuesta (en microsegundos)

// Multiplicador de seguridad para evitar colisiones si varios Anchors intentaran hablar a la vez. 
// En un sistema "dirigido" (polling) no es crítico, pero es una buena práctica de diseño.
#define ANCHOR_RESP_OFFSET_US 2000

#define ANCHOR_ID 3   // <<< Cambiar para cada anchor

// Configuración de radiofrecuencia del DW3000
// Debe ser exactamente igual a la del tag para que puedan entenderse
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


//Plantilla del poll
// El Anchor comparará los mensajes entrantes con esta plantilla para saber si es un Poll válido
static uint8_t rx_poll_msg_template[] = {
    0x41,0x88,0,0xCA,0xDE,'W','A','V','E',0xE0,0,0
};

// Mensaje de Respuesta del Anchor
// Mismo formato general, pero invierte "WAVE" a "VEWA" y ajusta las direcciones
static uint8_t tx_resp_msg[] = {
    0x41,0x88,0, ANCHOR_ID, 0xDE, 'V','E','W','A',0xE1,
    0,0,0,0,0,0,0,0,0,0 // Espacio vacío donde se insertarán los Timestamps luego
};

static uint8_t rx_buffer[20];    // Memoria para guardar el mensaje que llega
static uint8_t frame_seq_nb = 0; // Contador de secuencia para las respuestas
static uint32_t status_reg = 0;  // Variable para leer el estado del DW3000

static uint64_t poll_rx_ts; // Timestamp cuándo recibimos el Poll
static uint64_t resp_tx_ts; // Timestamp cuándo vamos a enviar la Respuesta

extern dwt_txconfig_t txconfig_options; // Carga la configuración de potencia de transmisión de la librería

// Inicialización
void setup()
{
  UART_init();              // Inicializa pines UART
  Serial.begin(115200);     // Abre el puerto serie para ver lo datos en la PC a 115200 baudios

  _fastSPI = SPISettings(16000000L, MSBFIRST, SPI_MODE0);  // Configura la velocidad del bus SPI a 16 MHz

  spiBegin(PIN_IRQ, PIN_RST);   // Inicializa el SPI con los pines de control
  spiSelect(PIN_SS);            // Asigna el pin de selección de esclavo
  delay(2);                     // Pausa de estabilización

  while (!dwt_checkidlerc()) { Serial.println("IDLE FAILED"); while(1); }   // Verifica que el DW3000 esté encendido y en modo reposo (Idle). Si falla, traba el programa (while(1)).
  if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) { Serial.println("INIT FAILED"); while(1); }   // Inicializa los registros base del DW3000

  dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);    // Enciende los LEDs integrados del chip

  if (dwt_configure(&config)) { Serial.println("CONFIG FAILED"); while(1); }   // Carga la configuración de radiofrecuencia que se declaró anteriormente

  dwt_configuretxrf(&txconfig_options); // Configura la potencia del transmisor de radio
  dwt_setrxantennadelay(RX_ANT_DLY);    // Aplica la compensación de retardo de la antena RX
  dwt_settxantennadelay(TX_ANT_DLY);    // Aplica la compensación de retardo de la antena TX

  // Activa el LNA (Low Noise Amplifier) para RX y el PA (Power Amplifier) para TX. Mejora el alcance.
  dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);


  Serial.print("ANCHOR iniciado con ID = ");
  Serial.println(ANCHOR_ID);
}

void loop() // Buvle principal
{
  // Pone al DW3000 en modo "Escucha"
  dwt_rxenable(DWT_START_RX_IMMEDIATE);

  // Bucle de espera: Lee el registro de estado hasta que llegue un mensaje o haya un error
  while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) &
           (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR))) {}

  // Revisa si llegó un mensaje sin errores
  if (status_reg & SYS_STATUS_RXFCG_BIT_MASK) 
  {
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK); // Limpia la bandera de "mensaje recibido" para el próximo ciclo

    uint32_t frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK; // Lee la longitud del mensaje entrante

    if (frame_len <= sizeof(rx_buffer))  // Se asegura de que no exceda el buffer
    {
      dwt_readrxdata(rx_buffer, frame_len, 0);  // Extrae el mensaje de la memoria del chip al array rx_buffer
      rx_buffer[2] = 0;  // Sobrescribe el byte de secuencia con 0 temporalmente para que no afecte la comparación (memcmp)

      // Verifica si el mensaje es exactamente la plantilla Poll que esperamos
      if (memcmp(rx_buffer, rx_poll_msg_template, 9) == 0)
      {
        uint8_t poll_dest = rx_buffer[9]; // Revisa el byte de "Destino" (byte 9)
        if (poll_dest == (uint8_t)(0xE0 + ANCHOR_ID))
        {
          // Prepara la respuesta
          uint32_t resp_tx_time;
          int ret;

          poll_rx_ts = get_rx_timestamp_u64();

          // Calcula CUÁNDO va a responder en el futuro.
          // Suma el tiempo de recepción + el delay base + el offset de seguridad por ID.
          // Se desplaza 8 bits a la derecha (>> 8) porque el chip DW3000 usa 32 bits para programar alarmas de transmisión.
          resp_tx_time = (poll_rx_ts + ((POLL_RX_TO_RESP_TX_DLY_UUS + ANCHOR_ID * ANCHOR_RESP_OFFSET_US) * UUS_TO_DWT_TIME)) >> 8;
          dwt_setdelayedtrxtime(resp_tx_time);

          resp_tx_ts = (((uint64_t)(resp_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY; // Calcula el Timestamp de TX final para mandárselo al Tag

          // Escribe los dos Timestamps dentro del mensaje de respuesta
          resp_msg_set_ts(&tx_resp_msg[10], poll_rx_ts);
          resp_msg_set_ts(&tx_resp_msg[14], resp_tx_ts);

          tx_resp_msg[2] = frame_seq_nb; // Escribe el número de secuencia actual en el mensaje

          // Escribe el mensaje completo en la RAM de transmisión del chip
          dwt_writetxdata(sizeof(tx_resp_msg), tx_resp_msg, 0);
          dwt_writetxfctrl(sizeof(tx_resp_msg), 0, 1);

          ret = dwt_starttx(DWT_START_TX_DELAYED); // Ejecuta la orden de Transmisión Retrasada. El chip esperará hasta que el reloj llegue a `resp_tx_time` y disparará la onda.

          if (ret == DWT_SUCCESS) // Comprueba que la orden se haya programado con éxito
          {
            while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK)) {} // Espera bloqueado hasta que el chip confirme que físicamente ya mandó el mensaje
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK); // Limpia la bandera de transmisión exitosa
            frame_seq_nb++;  // Prepara el número de secuencia para la próxima vez
            // Imprime un pequeño aviso de depuración por el puerto serie
            Serial.print("Anchor ");
            Serial.print(ANCHOR_ID);
            Serial.println(" respondió");
          }
        } 
      } 
    } 
  }
  else
  {
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);  // Si la interrupción fue por un error o ruido de RF, limpia los registros de error para seguir escuchando
  }
}
