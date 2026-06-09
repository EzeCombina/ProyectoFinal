import rclpy
import time
from rclpy.node import Node
from geometry_msgs.msg import Twist
from gpiozero import Motor, OutputDevice

# Controlador de Ruedas Mecanum 
class MecanumController(Node):
    def __init__(self):
        super().__init__('mecanum_controller')
        self.get_logger().info('Iniciando Controlador de Ruedas Mecanum Híbrido Protegido...')

        self.ultimo_comando_manual = 0.0    # Marca de tiempo de la última orden manual
        self.timeout_manual = 0.5           # Tiempo de gracia (segundos) para el modo manual

        # Canal Autónomo Original (Nav2 directo a través del remapeo del launch)
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)

        # Canal Manual Independiente desde la Web o Teclado
        self.sub_manual = self.create_subscription(
            Twist,
            '/cmd_vel_teleop',             # Tópico exclusivo para control manual
            self.manual_callback,
            10)

        # Configuración de los motores y el pin de standby
        self.stby = OutputDevice(17)
        self.stby.on() 

        self.fl_motor = Motor(forward=5, backward=6, enable=12, pwm=True)
        self.bl_motor = Motor(forward=16, backward=20, enable=13, pwm=True)
        self.fr_motor = Motor(forward=22, backward=23, enable=18, pwm=True)
        self.br_motor = Motor(forward=24, backward=25, enable=19, pwm=True)

    def manual_callback(self, msg):
        # Al recibir señal de la web o teclado, actualizamos el reloj e inyectamos movimiento
        self.ultimo_comando_manual = time.time()
        self.procesar_motores(msg)

    def cmd_vel_callback(self, msg):
        # Antes de ejecutar Nav2, se verifica si el usuario está operando manualmente
        if (time.time() - self.ultimo_comando_manual) < self.timeout_manual:
            return # Bloqueo temporal por seguridad
        self.procesar_motores(msg)

    def procesar_motores(self, msg):
        # Convierte la velocidad lineal y angular del mensaje Twist en velocidades individuales para cada motor
        vx = msg.linear.x  
        vy = msg.linear.y  
        omega = msg.angular.z  

        # Configuración de velocidades para cada motor según la cinemática de ruedas mecanum
        speed_fl = vx - vy - omega
        speed_fr = vx + vy + omega
        speed_bl = vx + vy - omega
        speed_br = vx - vy + omega

        # Normalización para asegurar que ninguna velocidad exceda el rango [-1, 1]
        max_speed = max(abs(speed_fl), abs(speed_fr), abs(speed_bl), abs(speed_br))
        if max_speed > 1.0:
            speed_fl /= max_speed
            speed_fr /= max_speed
            speed_bl /= max_speed
            speed_br /= max_speed

        self.set_motor_speed(self.fl_motor, speed_fl)
        self.set_motor_speed(self.fr_motor, speed_fr)
        self.set_motor_speed(self.bl_motor, speed_bl)
        self.set_motor_speed(self.br_motor, speed_br)

    # Envío de comandos a los motores con un mínimo de PWM para superar la fricción estática
    def set_motor_speed(self, motor, speed):
        min_pwm = 0.05 
        if abs(speed) > 0.05:
            out_speed = min_pwm + (abs(speed) * (1.0 - min_pwm))
            if speed > 0:
                motor.forward(out_speed)
            else:
                motor.backward(out_speed)
        else:
            motor.stop()

    # Función para detener todos los motores
    # Útil en caso de emergencia o al cerrar el nodo
    def stop_all(self):
        self.fl_motor.stop()
        self.fr_motor.stop()
        self.bl_motor.stop()
        self.br_motor.stop()
        self.stby.off()

def main(args=None):
    rclpy.init(args=args)
    nodo = MecanumController()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        nodo.get_logger().info('Deteniendo motores por interrupción del usuario...')
        nodo.stop_all()
    finally:
        nodo.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
