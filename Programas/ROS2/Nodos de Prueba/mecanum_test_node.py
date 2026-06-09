import rclpy
from rclpy.node import Node
from gpiozero import Motor, OutputDevice
import time

class MecanumTestNode(Node):
    def __init__(self):
        super().__init__('mecanum_test_node')
        self.get_logger().info('Iniciando Test de Motores Mecanum...')

        # Pin Standby (Habilita los chips TB6612FNG)
        self.stby = OutputDevice(17)
        self.stby.on() 

        # Configuración de motores usando gpiozero
        # Motor(forward, backward, enable=PWM)
        self.fl_motor = Motor(forward=5, backward=6, enable=12, pwm=True)
        self.bl_motor = Motor(forward=16, backward=20, enable=13, pwm=True)
        self.fr_motor = Motor(forward=22, backward=23, enable=18, pwm=True)
        self.br_motor = Motor(forward=24, backward=25, enable=19, pwm=True)

        # Variables de control
        self.speed = 0.3  # 50% de velocidad para probar seguro
        self.state = 0
        
        # Timer que cambia el movimiento cada 3 segundos
        self.timer = self.create_timer(3.0, self.secuencia_movimiento)

    def detener_todos(self):
        self.fl_motor.stop()
        self.bl_motor.stop()
        self.fr_motor.stop()
        self.br_motor.stop()

    def secuencia_movimiento(self):
        self.detener_todos()
        time.sleep(0.5) # Pausa mecánica para cuidar los engranajes

        if self.state == 0:
            self.get_logger().info('Movimiento: ADELANTE')
            self.fl_motor.forward(self.speed)
            self.bl_motor.forward(self.speed * 1.03)
            self.fr_motor.forward(self.speed)
            self.br_motor.forward(self.speed)

        elif self.state == 1:
            self.get_logger().info('Movimiento: ATRÁS')
            self.fl_motor.backward(self.speed)
            self.bl_motor.backward(self.speed)
            self.fr_motor.backward(self.speed)
            self.br_motor.backward(self.speed)

        elif self.state == 2:
            self.get_logger().info('Movimiento: LATERAL IZQUIERDA (Strafe)')
            self.fl_motor.backward(self.speed * 1.08)
            self.bl_motor.forward(self.speed * 0.85)
            self.fr_motor.forward(self.speed * 1.08)
            self.br_motor.backward(self.speed * 0.85)

        elif self.state == 3:
            self.get_logger().info('Movimiento: LATERAL DERECHA (Strafe)')
            self.fl_motor.forward(self.speed * 1.08)
            self.bl_motor.backward(self.speed * 0.85)
            self.fr_motor.backward(self.speed * 1.08)
            self.br_motor.forward(self.speed * 0.85)

        elif self.state == 4:
            self.get_logger().info('Movimiento: ROTACIÓN HORARIA')
            self.fl_motor.forward(self.speed)
            self.bl_motor.forward(self.speed)
            self.fr_motor.backward(self.speed)
            self.br_motor.backward(self.speed)

        elif self.state == 5:
            self.get_logger().info('Fin de la prueba. Deteniendo motores.')
            self.detener_todos()
            self.stby.off() # Poner drivers en bajo consumo
            self.timer.cancel() # Detener el timer

        #self.state += 1

def main(args=None):
    rclpy.init(args=args)
    nodo = MecanumTestNode()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        nodo.detener_todos()
        nodo.stby.off()
    finally:
        nodo.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()