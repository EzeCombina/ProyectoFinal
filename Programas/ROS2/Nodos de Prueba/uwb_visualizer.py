#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import json
import os

# Variables globales para compartir entre ROS 2 y Matplotlib
current_x, current_y = 0.0, 0.0
x_history, y_history = [], []

class VisualizerNode(Node):
    def __init__(self):
        super().__init__('uwb_visualizer')
        # Nos suscribimos al tópico que publica uwb_core.py
        self.subscription = self.create_subscription(
            PointStamped, '/uwb/raw_position', self.listener_callback, 10)
        self.get_logger().info("Visualizador suscrito a /uwb/raw_position")

    def listener_callback(self, msg):
        global current_x, current_y, x_history, y_history
        current_x = msg.point.x
        current_y = msg.point.y
        x_history.append(current_x)
        y_history.append(current_y)
        
        # Mantenemos una estela de los últimos 50 puntos
        if len(x_history) > 50:
            x_history.pop(0)
            y_history.pop(0)

# --- Configuración del Gráfico (Matplotlib) ---
config_path = os.path.expanduser('~/anchors_conf.json')
try:
    with open(config_path, 'r') as f:
        anchors = json.load(f)
except Exception as e:
    print(f"Aviso: No se encontró el JSON ({e}). Usando anclas por defecto.")
    anchors = {"1": [0.0, 0.0], "2": [5.0, 0.0], "3": [5.0, 5.0], "4": [0.0, 5.0]}

fig, ax = plt.subplots()
ax.set_title("Mapeo UWB en Tiempo Real (ROS 2)")
ax.set_xlabel("X (metros)")
ax.set_ylabel("Y (metros)")
ax.grid(True)
ax.set_aspect('equal')

# Dibujar anclas estáticas
max_x, max_y = 0.0, 0.0
for aid, pos in anchors.items():
    ax.plot(pos[0], pos[1], 'rs', markersize=10)
    ax.text(pos[0], pos[1] + 0.2, f"A{aid}", ha='center', fontweight='bold')
    max_x = max(max_x, pos[0])
    max_y = max(max_y, pos[1])

# Elementos dinámicos
tag_dot, = ax.plot([], [], 'bo', markersize=12, label="Robot TAG")
trail, = ax.plot([], [], 'b:', alpha=0.5)
ax.legend()

# Fijar límites del gráfico con un pequeño margen
ax.set_xlim(-1.0, max_x + 1.0)
ax.set_ylim(-1.0, max_y + 1.0)

node = None
ani = None # Declaramos la variable global arriba del main

def update(frame):
    global node
    if node:
        # Hacemos que ROS 2 procese los mensajes entrantes sin bloquear el gráfico
        rclpy.spin_once(node, timeout_sec=0.01)
        
    tag_dot.set_data([current_x], [current_y])
    trail.set_data(x_history, y_history)
    return tag_dot, trail

def main(args=None):
    global node, ani
    rclpy.init(args=args)
    node = VisualizerNode()
    
    # Agregamos cache_frame_data=False para evitar otros warnings de memoria
    ani = FuncAnimation(fig, update, interval=100, cache_frame_data=False)
    plt.show() # Bloquea la ejecución y muestra la ventana
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()