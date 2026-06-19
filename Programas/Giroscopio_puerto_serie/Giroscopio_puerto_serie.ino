//Librerías necesarias
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
#include <Preferences.h> // Para guardar los offsets permanentemente


//Creación de objetos
MPU6050 mpu;         //Objeto que representa al MPU6050
Preferences prefs;   //Objeto donde guardar la configuración del sensor

// Variables para el cálculo del ángulo
int16_t gz_raw;    //Lectura cruda
float gz_rad_s;    //Velocidad angular en radianes por segundo
float yaw = 0;     //Ángulo acumulado en radianes
long last_time;    //tiempo de última lectura

// Variables de calibración (Offsets)
int16_t ax_off, ay_off, az_off, gx_off, gy_off, gz_off;

void setup() {
    Serial.begin(115200); // abre puerto serie a 115200 baudios
    Wire.begin(21, 22);  //Establece el puerto I2C
    delay(500);  //pausa para que el sensor reciba energía

    // Iniciar memoria Preferences
    prefs.begin("mpu_offsets", false);
    cargarOffsets(); // Cargar los últimos offsets guardados

    Serial.println("INICIANDO_MPU");
    mpu.initialize();  //Configura el MPU

    // Verificación manual de conexión
    Wire.beginTransmission(0x68);
    if (Wire.endTransmission() != 0) {
        Serial.println("ERROR_CONEXION_I2C");
        while (1);
    }

    //Aplica la configuración de los offsets y arranca el reloj
    aplicarOffsets();
    last_time = micros();
}

void loop() {
    // Escucha si llegó algun dato ppor puerto serie
    if (Serial.available() > 0) {
        char c = Serial.read();    //Lee un caracter
        if (c == 'C') {           //Si le llegó una C ejecuta la calibracion del sensor
            ejecutarCalibracion();
        }
    }

    // Lee el valor curdo de rotación en el eje Z
    gz_raw = mpu.getRotationZ();
    
    // Convertir a radianes/segundo (escala 131.0 para +/- 250 deg/s)
    gz_rad_s = (gz_raw / 131.0) * (PI / 180.0);

    // Deadband (Filtro de ruido estático)
    if (abs(gz_rad_s) < 0.01) gz_rad_s = 0;

    // Integración temporal para obtener el ángulo Yaw
    long current_time = micros();
    float dt = (current_time - last_time) / 1000000.0;
    last_time = current_time;

    yaw += gz_rad_s * dt;  //Acumula el giro actual

    // Envía datos por puerto Serie
    Serial.print("YAW:");
    Serial.print(yaw, 4);
    Serial.print(",GZ:");
    Serial.println(gz_rad_s, 4);

    delay(100); // Frecuencia de envío de ~10Hz
}

//Función de calibración
void ejecutarCalibracion() {
    Serial.println("ESTADO:CALIBRANDO");
    // Usamos las funciones de autocalibración de la librería
    mpu.CalibrateAccel(6); //6 representa la cantidad de ciclos de ajuste
    mpu.CalibrateGyro(6);
    
    // Guardar los nuevos offsets encontrados
    gx_off = mpu.getXGyroOffset();
    gy_off = mpu.getYGyroOffset();
    gz_off = mpu.getZGyroOffset();
    ax_off = mpu.getXAccelOffset();
    ay_off = mpu.getYAccelOffset();
    az_off = mpu.getZAccelOffset();

    prefs.putShort("ax", ax_off);
    prefs.putShort("ay", ay_off);
    prefs.putShort("az", az_off);
    prefs.putShort("gx", gx_off);
    prefs.putShort("gy", gy_off);
    prefs.putShort("gz", gz_off);

    yaw = 0; // Reiniciamos el ángulo tras calibrar
    Serial.println("ESTADO:CALIBRACION_OK"); //Avisa por puerto serie que terminó de calibrar
    last_time = micros(); //reinicia reloj
}

//Funcion que lee la memoria permanente del ESP32 y carga los valores en las variables globales
void cargarOffsets() {
    ax_off = prefs.getShort("ax", 0);
    ay_off = prefs.getShort("ay", 0);
    az_off = prefs.getShort("az", 0);
    gx_off = prefs.getShort("gx", 0);
    gy_off = prefs.getShort("gy", 0);
    gz_off = prefs.getShort("gz", 0);
}

//Función que envía los comandos I2C para configurar los registros internos del MPU
void aplicarOffsets() {
    mpu.setXAccelOffset(ax_off);
    mpu.setYAccelOffset(ay_off);
    mpu.setZAccelOffset(az_off);
    mpu.setXGyroOffset(gx_off);
    mpu.setYGyroOffset(gy_off);
    mpu.setZGyroOffset(gz_off);
}