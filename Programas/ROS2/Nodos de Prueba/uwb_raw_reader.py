#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial

class UWBRawReaderNode(Node):
    def __init__(self):
        super().__init__('uwb_raw_reader')
        
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        
        port = self.get_parameter('serial_port').get_parameter_value().string_value
        baud = self.get_parameter('baud_rate').get_parameter_value().integer_value
        
        try:
            # Aumentamos el timeout a 1 segundo. 
            # Como usamos readline(), no va a esperar 1s si el mensaje llega rápido, 
            # solo evita que se corte a la mitad.
            self.ser = serial.Serial(port, baud, timeout=1.0)
            #self.ser.dtr = False
            #self.ser.rts = False
            self.ser.reset_input_buffer()
            self.get_logger().info(f'=== Monitor Crudo UWB iniciado en {port} a {baud} baudios ===')
        except serial.SerialException as e:
            self.get_logger().error(f'Error abriendo el puerto serial: {e}')
            raise SystemExit

        # Timer a 20 Hz
        self.timer = self.create_timer(0.05, self.read_serial_data)

    def read_serial_data(self):
        if not (self.ser and self.ser.is_open):
            return

        try:
            # Leemos MIENTRAS haya datos en el buffer, línea por línea
            while self.ser.in_waiting > 0:
                line = self.ser.readline()
                decoded_line = line.decode('utf-8', errors='ignore').strip()
                
                if decoded_line:
                    self.get_logger().info(f'RAW -> {decoded_line}')
                        
        except Exception as e:
            self.get_logger().warn(f'Ruido en la lectura: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = UWBRawReaderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ser:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()