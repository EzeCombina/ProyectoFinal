#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped # Mensaje para enviar un punto (x,y,z)
import serial # Librería para leer el puerto USB
import time

# Ctrl + K + C para comentar varias líneas en VSCode
# Ctrl + K + U para descomentar varias líneas en VSCode

class UWBDriverNode(Node):
    def __init__(self):
        super().__init__('uwb_driver')
        
        # --- CONFIGURACIÓN ---
        # Declaramos parámetros para poder cambiarlos sin tocar el código
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)

        # Obtenemos los valores de los parámetros
        port = self.get_parameter('serial_port').get_parameter_value().string_value
        baud = self.get_parameter('baud_rate').get_parameter_value().integer_value

        # --- PUBLICADOR ROS ---
        # Publicaremos la posición cruda en el tópico '/uwb/raw_position'
        # Usamos PointStamped porque incluye un header con tiempo y frame_id
        self.publisher_ = self.create_publisher(PointStamped, '/uwb/raw_position', 10)

        # --- CONEXIÓN SERIAL ---
        self.serial_conn = None
        self.connect_serial(port, baud)

        # Crear un timer que ejecute la lectura cada 0.1 segundos (10 Hz)
        self.timer = self.create_timer(0.1, self.read_uwb_data)
        
        self.get_logger().info(f'Nodo UWB iniciado en {port} a {baud} baudios.')

    def connect_serial(self, port, baud):
        """Intenta conectar con el ESP32"""
        try:
            self.serial_conn = serial.Serial(port, baud, timeout=1)

            # --- AGREGAR ESTAS DOS LÍNEAS ---
            # Esto evita que el ESP32 se quede trabado en modo bootloader o reinicio constante
            self.serial_conn.dtr = False 
            self.serial_conn.rts = False
            # --------------------------------

            self.serial_conn.reset_input_buffer()
        except serial.SerialException as e:
            self.get_logger().error(f'Error al abrir puerto serial: {e}')


    """Variable principal del nodo: lectura y publicación de datos UWB (No limpia el buffer)"""
    # def read_uwb_data(self):
    #     """Función principal: Lee serial, parsea y publica en ROS"""
    #     if self.serial_conn and self.serial_conn.is_open:
    #         try:
    #             if self.serial_conn.in_waiting > 0:
    #                 # Leemos la línea que manda el ESP32 (decodeamos bytes a string)
    #                 line = self.serial_conn.readline().decode('utf-8').strip()
                    
    #                 # ASUMIMOS FORMATO DEL ESP32: "POS:1.50,3.20,0.00"
    #                 # Esto deberás ajustarlo según cómo mande los datos tu ESP32
    #                 if line.startswith("POS:"):
    #                     data_str = line.replace("POS:", "")
    #                     coords = data_str.split(',')
                        
    #                     if len(coords) == 3:
    #                         x = float(coords[0])
    #                         y = float(coords[1])
    #                         z = float(coords[2])

    #                         # Crear mensaje ROS
    #                         msg = PointStamped()
    #                         msg.header.stamp = self.get_clock().now().to_msg()
    #                         msg.header.frame_id = "map" # Referencia global
    #                         msg.point.x = x
    #                         msg.point.y = y
    #                         msg.point.z = z

    #                         # Publicar mensaje
    #                         self.publisher_.publish(msg)
    #                         # self.get_logger().info(f'UWB Raw: x={x:.2f}, y={y:.2f}')

    #         except Exception as e:
    #             self.get_logger().warn(f'Error leyendo datos UWB: {e}')
    #     else:
    #         # Si se desconectó, intentar reconectar (opcional)
    #         pass

    """Variable principal del nodo: lectura y publicación de datos UWB (Limpia el buffer)"""
    def read_uwb_data(self):
        """Lee TODOS los datos pendientes y procesa solo el último"""
        if self.serial_conn and self.serial_conn.is_open:
            try:
                if self.serial_conn.in_waiting > 0:
                    # 1. Leemos TODO lo que hay en el buffer hasta dejarlo vacío
                    #    y nos quedamos solo con la última línea completa.
                    lines = self.serial_conn.read_all().decode('utf-8', errors='ignore').split('\n')
                    
                    # Filtramos líneas vacías y buscamos la última que tenga datos
                    last_valid_line = None
                    for line in reversed(lines):
                        if line.strip().startswith("POS:"):
                            last_valid_line = line.strip()
                            break
                    
                    # 2. Si encontramos una línea válida reciente, la procesamos
                    if last_valid_line:
                        # self.get_logger().info(f"Procesando: '{last_valid_line}'") # Debug
                        
                        data_str = last_valid_line.replace("POS:", "")
                        coords = data_str.split(',')
                        
                        if len(coords) == 3:
                            x = float(coords[0])
                            y = float(coords[1])
                            z = float(coords[2])

                            # Crear mensaje ROS
                            msg = PointStamped()
                            msg.header.stamp = self.get_clock().now().to_msg()
                            msg.header.frame_id = "map" 
                            msg.point.x = x
                            msg.point.y = y
                            msg.point.z = z

                            self.publisher_.publish(msg)

            except Exception as e:
                self.get_logger().warn(f'Error leyendo datos UWB: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = UWBDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.serial_conn:
            node.serial_conn.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()