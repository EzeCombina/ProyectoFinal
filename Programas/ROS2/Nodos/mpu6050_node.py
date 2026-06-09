#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import String # Importamos String para el estado
from std_srvs.srv import Trigger 
import serial 
import math
import transforms3d

# Nodo puente entre el MPU6050 en el ESP32 y ROS2. Publica datos IMU y maneja la calibración.
class IMUBridgeNode(Node):
    def __init__(self):
        super().__init__('mpu6050_node')
        # Publicador para los datos de IMU 
        self.publisher_ = self.create_publisher(Imu, '/imu/data', 10)
        
        # Publicador para avisarle a la web cuándo termina la calibración
        self.status_pub = self.create_publisher(String, '/imu/calibration_status', 10)
        
        self.is_calibrating = False
        
        # Intentamos abrir el puerto serie para comunicarnos con el ESP32
        try:
            self.ser = serial.Serial('/dev/ttyIMU', 115200, timeout=0.1)
            self.get_logger().info('Conexión serie establecida con ESP32.')
        except Exception as e:
            self.get_logger().error(f'No se pudo abrir el puerto serie: {e}')

        # Creamos el servicio para iniciar la calibración desde la web
        self.srv = self.create_service(Trigger, 'calibrate_imu', self.calibrate_imu_callback)
        self.get_logger().info('Servicio /calibrate_imu listo.')

        # Timer para leer continuamente el puerto serie sin bloquear el nodo
        self.timer = self.create_timer(0.01, self.timer_callback)

    # Callback del servicio de calibración. Se activa al presionar el botón en la web y envía el comando al ESP32.
    def calibrate_imu_callback(self, request, response):
        self.get_logger().info('Enviando comando de calibración al ESP32...')
        try:
            self.ser.write(b'C') 
            response.success = True
            response.message = "Comando 'C' enviado. Iniciando proceso..."
        except Exception as e:
            response.success = False
            response.message = f"Error UART: {e}"
        
        return response # Responde al instante a la web para evitar el timeout de Rosbridge

    # Timer callback para leer datos del ESP32 sin bloquear el nodo. Procesa tanto estados de calibración como datos IMU.
    def timer_callback(self):
        if self.ser.in_waiting > 0:
            try:
                line = self.ser.readline().decode('utf-8').strip()

                if "ESTADO:CALIBRANDO" in line:
                    if not self.is_calibrating:
                        self.get_logger().info("ESP32 está calibrando... No mover el robot.")
                        self.is_calibrating = True
                        # Avisamos a la web que empezó
                        msg_status = String()
                        msg_status.data = "CALIBRANDO"
                        self.status_pub.publish(msg_status)
                    
                elif "ESTADO:CALIBRACION_OK" in line:
                    self.get_logger().info("Calibración finalizada con éxito.")
                    self.is_calibrating = False
                    
                    # Publicamos el OK definitivo. La web va a capturar esto para reactivar la UI
                    msg_status = String()
                    msg_status.data = "OK"
                    self.status_pub.publish(msg_status)
                    
                # Si no estamos calibrando, procesamos los datos de orientación y velocidad angular que vienen del ESP32
                elif not self.is_calibrating and "YAW:" in line:
                    parts = line.split(',') 
                    if len(parts) == 2:
                        yaw_raw = float(parts[0].split(':')[1])
                        gz_rad = float(parts[1].split(':')[1])

                        msg = Imu()
                        msg.header.stamp = self.get_clock().now().to_msg()
                        msg.header.frame_id = 'imu_link'

                        # Convertimos el ángulo de yaw a cuaterniones para llenar el mensaje IMU
                        quat = transforms3d.euler.euler2quat(0, 0, yaw_raw)
                        msg.orientation.w = quat[0]
                        msg.orientation.x = quat[1]
                        msg.orientation.y = quat[2]
                        msg.orientation.z = quat[3]
                        
                        # Para simplificar, asumimos que solo nos interesa la velocidad angular en Z (yaw rate)
                        msg.angular_velocity.z = gz_rad
                        
                        # Asignamos covarianzas bajas para indicar que confiamos en estos datos
                        msg.orientation_covariance[8] = 0.01 
                        msg.angular_velocity_covariance[8] = 0.01

                        self.publisher_.publish(msg)

            except Exception as e:
                pass

def main(args=None):
    rclpy.init(args=args)
    node = IMUBridgeNode()
    try:
        rclpy.spin(node) # Volvemos al spin común, ya no necesitamos multihilo remoto!
    except KeyboardInterrupt:
        node.get_logger().info('Deteniendo nodo IMU...')
    finally:
        node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()