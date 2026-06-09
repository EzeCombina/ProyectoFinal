#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from gpiozero import Button

class ButtonCalibratorNode(Node):
    def __init__(self):
        super().__init__('button_calibrator')
        
        # Cliente del servicio de calibración
        self.cli = self.create_client(Trigger, '/calibrate_uwb')
        
        # Configurar el botón en el GPIO 17 (Pin 11 físico de la Raspberry Pi)
        # pull_up=True activa la resistencia interna. 
        # bounce_time=0.1 evita lecturas fantasma (anti-rebote por software)
        self.button = Button(17, pull_up=True, bounce_time=0.1)
        
        # Interrupción: cuando se presione, llama a la función send_request
        self.button.when_pressed = self.send_request
        
        self.get_logger().info('Lector de botón iniciado. Esperando pulsación en GPIO 17...')

    def send_request(self):
        # Verifica si el nodo uwb_core está corriendo y escuchando
        if not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Servicio /calibrate_uwb no disponible. ¿Está corriendo uwb_core?')
            return
            
        self.get_logger().info('¡Botón presionado! Solicitando calibración...')
        
        # Armar y enviar la petición de forma asíncrona para no congelar el nodo
        req = Trigger.Request()
        future = self.cli.call_async(req)
        future.add_done_callback(self.callback_response)

    def callback_response(self, future):
        try:
            # Leer qué nos contestó uwb_core.py
            response = future.result()
            if response.success:
                self.get_logger().info(f'ÉXITO: {response.message}')
            else:
                self.get_logger().error(f'FALLÓ: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Error al procesar la respuesta del servicio: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = ButtonCalibratorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()