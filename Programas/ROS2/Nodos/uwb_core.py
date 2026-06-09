#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PointStamped, PoseWithCovarianceStamped
from std_srvs.srv import Trigger  
from std_msgs.msg import String
import serial
import time
import re
import math
import json
import os
import numpy as np
from scipy.optimize import least_squares
from collections import deque
from rclpy.qos import QoSProfile, DurabilityPolicy

# ========= CONFIGURACIÓN DE ALTURA PARA PROYECCIÓN =========
ALTURA_ANCHORS = 0.06
ALTURA_TAG = 0.15
DELTA_Z = ALTURA_ANCHORS - ALTURA_TAG  

# ========= Función de predcción =========
# Se intenta encontrar el error residual de la trilateración.
# Mide qué tan lejos está una coordenada hipotética de ser la posición real del robot.
# La función toma como entrada una suposición de posición (guess), las posiciones de las anclas y las distancias medidas, 
# y devuelve un array con las diferencias para cada ancla. El optimizador intentará encontrar el guess que minimice estas diferencias.
def eq_loss(guess, positions, distances):
    res = []
    for (ax, ay), d in zip(positions, distances):
        res.append(np.sqrt((guess[0] - ax) ** 2 + (guess[1] - ay) ** 2) - d)
    return res

# ======== NODO PRINCIPAL ========
class UWBCoreNode(Node):
    # Función de inicialización del nodo, donde se configuran parámetros, se cargan anclas, se crean publishers y servicios, y se establece la conexión serial con el ESP32.
    def __init__(self):
        super().__init__('uwb_core')
        
        # Parámetros configurables para el nodo, incluyendo puerto serial, baud rate, cantidad de muestras para calibración, y umbral para filtrado de outliers.
        self.declare_parameter('serial_port', '/dev/ttyUWB')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('calib_samples', 100)
        self.declare_parameter('outlier_threshold', 0.20)
        
        self.port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baud = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self.samples = self.get_parameter('calib_samples').get_parameter_value().integer_value
        self.threshold = self.get_parameter('outlier_threshold').get_parameter_value().double_value
        
        # Ruta al archivo de configuración de anclas
        self.config_path = os.path.expanduser('~/anchors_conf.json')
        
        self.is_calibrating = False
        self.anchors = {}
        self.load_anchors()

        self.min_anchors = 3
        self.window_size = 3
        
        self.jump_warnings = 0
        self.max_jump = 0.15

        self.current_x = sum([pos[0] for pos in self.anchors.values()]) / 4.0 if self.anchors else 0.0
        self.current_y = sum([pos[1] for pos in self.anchors.values()]) / 4.0 if self.anchors else 0.0
        
        # Qos para el tamaño del dominio, queremos que los late joiners lo reciban aunque se publique antes de que se unan
        qos_profile = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        # Publicador para la posición cruda (sin covarianza) que usará la web
        self.publisher_ = self.create_publisher(PointStamped, '/uwb/raw_position', 10)
        
        # Publicador para la posición con covarianza que usará el EKF
        self.ekf_publisher = self.create_publisher(PoseWithCovarianceStamped, '/uwb/pose', 10)
        
        # Servicio para iniciar la calibración, que se puede llamar desde la terminal o desde un botón en la web
        self.calib_service = self.create_service(Trigger, '/calibrate_uwb', self.calibrate_callback)

        # Publicador para el tamaño del dominio, con QoS Transient Local para que nuevos suscriptores lo reciban aunque se publique antes de que se unan
        self.domain_pub = self.create_publisher(Point, '/uwb/domain_size', qos_profile)

        # Publicador para avisarle a la web cuándo termina la calibración UWB
        self.uwb_status_pub = self.create_publisher(String, '/uwb/calibration_status', 10)
        
        # Establecemos la conexión serial con el ESP32, configurando los pines DTR y RTS para evitar resets no deseados, y limpiando el buffer de entrada antes de comenzar a leer datos
        try:
            # self.ser = Conexión Serial
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
            self.ser.dtr = False
            self.ser.rts = False
            self.ser.reset_input_buffer()
            self.get_logger().info(f'Nodo UWB iniciado en {self.port}.')
        except serial.SerialException as e:
            self.get_logger().error(f'Error abriendo puerto: {e}')
            raise SystemExit

        # Timer para procesar los datos del UWB 
        self.timer = self.create_timer(0.1, self.process_uwb_data)

    # Función para cargar la configuración de las anclas desde un archivo JSON.
    def load_anchors(self):
        try:
            with open(self.config_path, 'r') as f:
                raw_anchors = json.load(f)
                self.anchors = {int(k): v for k, v in raw_anchors.items()}
            self.get_logger().info(f"Configuración cargada: {self.anchors}")
        except FileNotFoundError:
            # Si no existe el archivo, se crean anclas por defecto en las esquinas de un área de 5x5 metros.
            self.get_logger().warn("No hay JSON de configuración. Se usarán valores por defecto 5x5m. ¡EJECUTAR CALIBRACIÓN!")
            self.anchors = {1: [0.0, 0.0], 2: [5.0, 0.0], 3: [5.0, 5.0], 4: [0.0, 5.0]}

    # Función para filtrar outliers y promediar las lecturas de distancia durante la calibración. 
    # Se utiliza la mediana para identificar el valor central, y luego se eliminan las lecturas que están demasiado lejos de esa mediana antes de calcular el promedio final.
    def filter_and_average(self, data_list):
        if not data_list: return 0.0
        arr = np.array(data_list)
        median_val = np.median(arr)
        clean_arr = arr[np.abs(arr - median_val) < self.threshold]
        if len(clean_arr) == 0: clean_arr = arr
        return np.mean(clean_arr)

    # Función callback para el servicio de calibración, que se activa cuando se llama al servicio /calibrate_uwb. 
    # Esta función inicia el proceso de calibración, limpia el buffer de entrada del serial, y publica un mensaje inmediato a la web para indicar que la calibración ha comenzado. 
    def calibrate_callback(self, request, response):
        self.get_logger().info("=== INICIANDO CALIBRACIÓN UWB ===")
        self.is_calibrating = True
        self.ser.reset_input_buffer()
        
        # Avisamos inmediatamente a la web por el tópico que el proceso comenzó
        msg_status = String()
        msg_status.data = "CALIBRANDO"
        self.uwb_status_pub.publish(msg_status)
        
        # Inicializamos las estructuras para almacenar las lecturas de calibración
        self.calib_readings = {1: [], 2: [], 3: [], 4: []}
        self.calib_count = 0
        self.calib_timeout_start = time.time()
        
        # Respondemos de inmediato a Rosbridge para anular el timeout de 5s
        response.success = True
        response.message = "Proceso de calibración UWB iniciado en hardware."
        return response

    # Función principal para procesar los datos del UWB. 
    # Esta función se ejecuta periódicamente gracias al timer, y maneja tanto el proceso de calibración como la lectura normal de datos para posicionamiento.
    def process_uwb_data(self):
        # Si no existe una conexión serial válida no se hace nada
        if not (self.ser and self.ser.is_open):
            return

        # Si se encuentra en modo calibración se procesan las lecturas de distancia para calcular la 
        # geometría de la habitación y actualizar la configuración de anclas, publicando el resultado a la web.
        if self.is_calibrating:
            msg_status = String()
            
            # Control de Timeout de seguridad (30 segundos)
            if time.time() - self.calib_timeout_start > 30.0:  
                self.is_calibrating = False
                msg_status.data = "ERROR:Timeout esperando datos del ESP32."
                self.uwb_status_pub.publish(msg_status)
                return
        
            # Se lee el buffer serial
            if self.ser.in_waiting > 0:
                try:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    matches = re.findall(r'A(\d):\s*([-+]?[0-9]*\.?[0-9]+)', line) 
                    found_any = False
                    for aid_str, dist_str in matches:
                        # aid es el ID del anchor, dist es la distancia medida.
                        aid, dist = int(aid_str), float(dist_str)
                        if dist > abs(DELTA_Z):
                            dist_proyectada = math.sqrt(dist**2 - abs(DELTA_Z)**2)
                            self.calib_readings[aid].append(dist_proyectada)
                            found_any = True
                    if found_any: 
                        self.calib_count += 1

                    # Si se completan las muestras requeridas, se procesa la geometría
                    if self.calib_count >= self.samples:
                        dists = {aid: self.filter_and_average(vals) if vals else 0.0 for aid, vals in self.calib_readings.items()}
                        d1, d2, d4 = dists.get(1, 0), dists.get(2, 0), dists.get(4, 0)
                        
                        # Se valida que no falten datos críticos
                        if 0 in (d1, d2, d4):
                            self.is_calibrating = False
                            msg_status.data = "ERROR:Faltan datos de anchors críticos."
                            self.uwb_status_pub.publish(msg_status)
                            return

                        # Se calcula la geometría de la habitación usando las distancias medidas, 
                        # y se actualiza el archivo de configuración de anclas con las nuevas posiciones. 
                        room_width = d1 + d2
                        if d4 > d1:
                            room_height = math.sqrt(d4 ** 2 - d1 ** 2)
                            new_anchors = {
                                "1": [0.0, 0.0],
                                "2": [round(room_width, 3), 0.0],
                                "3": [round(room_width, 3), round(room_height, 3)],
                                "4": [0.0, round(room_height, 3)]
                            }
                            with open(self.config_path, 'w') as f:
                                json.dump(new_anchors, f, indent=4)
                            self.load_anchors()
                            
                            # Se publica el tamaño del dominio para la web
                            domain_msg = Point()
                            domain_msg.x = float(room_width)    
                            domain_msg.y = float(room_height)   
                            domain_msg.z = 0.0                  
                            self.domain_pub.publish(domain_msg)

                            self.is_calibrating = False
                            # Enviamos el mensaje de éxito definitivo por el tópico
                            msg_status.data = f"OK:Calibración OK: {room_width:.2f}m x {room_height:.2f}m"
                            self.uwb_status_pub.publish(msg_status)
                        else:
                            self.is_calibrating = False
                            msg_status.data = "ERROR:Error geométrico. ¿Robot mal posicionado?"
                            self.uwb_status_pub.publish(msg_status)
                except Exception as e:
                    pass
            return

        # Procesamiento de lecturas normales para el posicionamiento 
        try:
            if self.ser.in_waiting > 0:
                lines = self.ser.read_all().decode('utf-8', errors='ignore').split('\n')
                last_valid_line = None
                # Se busca la última línea valida que contenga datos de anclas
                for line in reversed(lines):
                    if line.strip().startswith("A"):
                        last_valid_line = line.strip()
                        break
                
                # Si no se encuentra ninguna línea válida, se sale de la función para esperar la próxima lectura
                if not last_valid_line: return

                # Se extraen las distacias 
                matches = re.findall(r'A(\d):\s*([-+]?[0-9]*\.?[0-9]+)', last_valid_line)
                
                # Se verifica si se tienen suficioentes datos de anclas 
                if len(matches) >= self.min_anchors:
                    # Se crea un diccionario con los ID de ancla y sus distancias medidas
                    dists_dict = {int(id): float(dist) for id, dist in matches}
                    valid_anchors_pos = []
                    valid_dists = []

                    # Se proyectan las distancias para cada ancla y se filtran las que no sean válidas
                    for aid in self.anchors:
                        if aid in dists_dict and dists_dict[aid] > DELTA_Z:
                            dist_cruda = dists_dict[aid]
                            dist_proyectada = math.sqrt(dist_cruda**2 - DELTA_Z**2)
                            valid_anchors_pos.append(self.anchors[aid])
                            valid_dists.append(dist_proyectada) 

                    # Se calcula la posición del robot usando trilateración no lineal
                    if len(valid_dists) >= 3:
                        # initial_guess es la posición actual del robot, 
                        # lo que ayuda a que el optimizador converja más rápido y evita saltos bruscos
                        initial_guess = [self.current_x, self.current_y]
                        res = least_squares(eq_loss, initial_guess, args=(valid_anchors_pos, valid_dists), bounds=([0.0, 0.0], [float('inf'), float('inf')]))
                        raw_x, raw_y = res.x[0], res.x[1]

                        # current_x y current_y guardan la última posición "estable" del robot.
                        # raw_x y raw_y son la nueva posición calculada a partir de las distancias medidas.
                        # Se calcula la distancia entre la nueva posición y la última posición estable para detectar posibles saltos bruscos (outliers).
                        dist_jump = math.hypot(raw_x - self.current_x, raw_y - self.current_y)

                        # Si la distancia del salto es mayor al humbral, se incrementa el contador de advertencias. 
                        # Si el contador supera un cierto límite, se ignora la nueva posición y se mantiene la última posición estable.
                        if dist_jump > self.max_jump:
                            self.jump_warnings += 1
                            if self.jump_warnings < 1:
                                return
                            else:
                                self.jump_warnings = 0
                        else:
                            self.jump_warnings = 0

                        # Preferencias para la página web 
                        if dist_jump < 0.08:
                            raw_x = self.current_x
                            raw_y = self.current_y

                        # Filtro dinámico: Si el salto es pequeño, se confía más en la nueva lectura (alpha bajo). 
                        # Si el salto es grande, se confía más en la posición anterior (alpha alto).
                        if dist_jump < 0.20:
                            dynamic_alpha = 0.15
                        else:
                            dynamic_alpha = 0.4

                        smooth_x = dynamic_alpha * raw_x + (1 - dynamic_alpha) * self.current_x
                        smooth_y = dynamic_alpha * raw_y + (1 - dynamic_alpha) * self.current_y

                        # Actualizamos la posición actual estable con la nueva posición suavizada
                        # para el siguiente ciclo
                        self.current_x, self.current_y = smooth_x, smooth_y

                        # Se publica la posición para la web 
                        msg_web = PointStamped()
                        msg_web.header.stamp = self.get_clock().now().to_msg()
                        msg_web.header.frame_id = "odom"
                        msg_web.point.x = float(smooth_x)
                        msg_web.point.y = float(smooth_y)
                        msg_web.point.z = 0.0
                        self.publisher_.publish(msg_web)
                        
                        # Se publica la posición con covarianza para el EKF
                        msg_ekf = PoseWithCovarianceStamped()
                        msg_ekf.header.stamp = self.get_clock().now().to_msg()
                        msg_ekf.header.frame_id = "odom" 
                        msg_ekf.pose.pose.position.x = float(smooth_x)
                        msg_ekf.pose.pose.position.y = float(smooth_y)
                        msg_ekf.pose.pose.position.z = 0.0
                        
                        # Definimos qué tanta confianza le tenemos al UWB (0.05 metros de varianza aprox)
                        msg_ekf.pose.covariance[0] = 0.05  # Varianza en X
                        msg_ekf.pose.covariance[7] = 0.05  # Varianza en Y
                        
                        self.ekf_publisher.publish(msg_ekf)

        except Exception as e:
            pass 

def main(args=None):
    rclpy.init(args=args)
    node = UWBCoreNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ser: node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
