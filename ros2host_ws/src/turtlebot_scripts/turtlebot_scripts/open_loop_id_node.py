#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Nodo: open_loop_id_node
-----------------------------------
- Realiza pruebas en lazo abierto para identificar la planta (lineal y angular)
- Aplica una señal conocida (escalón, rampa, PRBS)
- Guarda CSV con t, v_cmd, w_cmd, x, y, yaw, v_real, w_real
- 🚨 Seguridad: si se presiona el bumper, detiene el robot y guarda los datos
"""

import os
import csv
import time
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from kobuki_ros_interfaces.msg import BumperEvent

class OpenLoopID(Node):
    def __init__(self):
        super().__init__('open_loop_id_node')

        # ----------------------------
        #  🔧 PARÁMETROS CONFIGURABLES
        # ----------------------------
        self.declare_parameter('mode', 'angular')      # 'lineal' o 'angular'
        self.declare_parameter('signal', 'step')      # 'step', 'ramp', 'prbs'
        self.declare_parameter('amplitude', 2.0)      # m/s o rad/s
        self.declare_parameter('duration', 12.0)      # segundos
        self.declare_parameter('csv_dir', '/ros2_ws/ros2host_ws/src/turtlebot_scripts/turtlebot_scripts')

        self.mode = self.get_parameter('mode').value
        self.signal = self.get_parameter('signal').value
        self.amplitude = float(self.get_parameter('amplitude').value)
        self.duration = float(self.get_parameter('duration').value)
        self.csv_dir = self.get_parameter('csv_dir').value

        # ----------------------------
        #  📡 PUBLICADORES / SUSCRIPTORES
        # ----------------------------
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom_raw', self.odom_callback, 10)
        self.create_subscription(BumperEvent, '/events/bumper', self.bumper_callback, 10)

        # ----------------------------
        #  🧮 VARIABLES INTERNAS
        # ----------------------------
        self.start_time = time.time()
        self.last_time = self.start_time
        self.x = self.y = self.yaw = 0.0
        self.v_real = self.w_real = 0.0
        self.last_x = self.last_y = self.last_yaw = 0.0
        self.bumper_pressed = False
        self.data = []

        # ----------------------------
        #  💾 CSV
        # ----------------------------
        os.makedirs(self.csv_dir, exist_ok=True)
        timestamp = time.strftime('%Y-%m-%d_%H-%M-%S')
        self.csv_path = os.path.join(self.csv_dir, f'data_openloop_{self.mode}_{timestamp}.csv')
        self.get_logger().info(f'📁 Guardando datos en: {self.csv_path}')

        # ----------------------------
        #  ⏱️ TIMER PRINCIPAL (20 Hz)
        # ----------------------------
        self.timer = self.create_timer(0.05, self.update)

    # -----------------------------------------------------
    # 🧭 CALLBACK ODOMETRÍA
    # -----------------------------------------------------
    def odom_callback(self, msg):
        # 1) timestamp estable del mensaje
        stamp = msg.header.stamp
        now = stamp.sec + stamp.nanosec * 1e-9
        dt = now - self.last_time if hasattr(self, "last_time") else 0.0
        if dt <= 0:
            dt = 1e-3  # evita división por cero en primera muestra

        # 2) pose -> yaw
        q = msg.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))

        # 3) "unwrap": mantiene continuidad del ángulo
        if not hasattr(self, "yaw_unwrap"):
            self.yaw_unwrap = yaw
        else:
            dyaw_wrapped = yaw - self.last_yaw
            # normaliza el salto a (-pi, pi]
            dyaw = math.atan2(math.sin(dyaw_wrapped), math.cos(dyaw_wrapped))
            self.yaw_unwrap += dyaw

        # 4) velocidad angular: usa la del mensaje si existe; si no, deriva la yaw desenvolvida
        if abs(msg.twist.twist.angular.z) > 1e-6:
            w_meas = msg.twist.twist.angular.z
        else:
            w_meas = (self.yaw_unwrap - getattr(self, "last_yaw_unwrap", self.yaw_unwrap)) / dt

        # 5) velocidad lineal desde twist si está; si no, deriva posición
        vx_msg = msg.twist.twist.linear.x
        if abs(vx_msg) > 1e-6:
            v_meas = vx_msg
        else:
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            dx = x - getattr(self, "last_x", x)
            dy = y - getattr(self, "last_y", y)
            v_meas = math.hypot(dx, dy) / dt

        # 6) anti-picos: recorte + filtro simple
        # (opcional: ajusta límites a tu plataforma)
        w_meas = max(min(w_meas, 5.0), -5.0)  # recorta a ±5 rad/s
        alpha = 0.2  # filtro EMA suave
        if hasattr(self, "w_real"):
            self.w_real = (1-alpha)*self.w_real + alpha*w_meas
            self.v_real = (1-alpha)*self.v_real + alpha*v_meas
        else:
            self.w_real = w_meas
            self.v_real = v_meas

        # actualizar estados
        self.last_time = now
        self.last_yaw = yaw
        self.last_yaw_unwrap = self.yaw_unwrap
        self.last_x = msg.pose.pose.position.x
        self.last_y = msg.pose.pose.position.y
        self.yaw = yaw


    # -----------------------------------------------------
    # 🚨 CALLBACK BUMPER
    # -----------------------------------------------------
    def bumper_callback(self, msg: BumperEvent):
        if msg.state == BumperEvent.PRESSED:
            self.get_logger().warn('🚨 BUMPER ACTIVADO — deteniendo robot y finalizando prueba')
            self.bumper_pressed = True
            self.stop_and_save()

    # -----------------------------------------------------
    # 🔁 LOOP PRINCIPAL
    # -----------------------------------------------------
    def update(self):
        # Si bumper fue presionado, no continuar
        if self.bumper_pressed:
            return

        t = time.time() - self.start_time
        cmd = Twist()

        # --- Generar señal ---
        if self.signal == 'step':
            u = self.amplitude if t < self.duration else 0.0
        elif self.signal == 'ramp':
            u = min(self.amplitude * (t / self.duration), self.amplitude)
        else:
            u = 0.0  # (PRBS u otras señales se pueden agregar luego)

        # --- Aplicar según modo ---
        if self.mode == 'lineal':
            cmd.linear.x = u
            cmd.angular.z = 0.0
        else:
            cmd.linear.x = 0.0
            cmd.angular.z = u

        # Publicar comando
        self.cmd_pub.publish(cmd)

        # Guardar datos
        self.data.append([
            round(t, 3),
            round(cmd.linear.x, 3),
            round(cmd.angular.z, 3),
            round(self.x, 4),
            round(self.y, 4),
            round(self.yaw, 4),
            round(self.v_real, 4),
            round(self.w_real, 4)
        ])

        # Terminar automáticamente cuando termina la prueba
        if t >= (self.duration + 2):
            self.get_logger().info('✅ Prueba completada correctamente.')
            self.stop_and_save()

    # -----------------------------------------------------
    # 🧩 FUNCIÓN DE CIERRE Y GUARDADO
    # -----------------------------------------------------
    def stop_and_save(self):
        # Detener el robot
        stop_msg = Twist()
        self.cmd_pub.publish(stop_msg)
        time.sleep(0.5)
        self.cmd_pub.publish(stop_msg)

        # Guardar CSV
        with open(self.csv_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['t', 'v_cmd', 'w_cmd', 'x', 'y', 'yaw', 'v_real', 'w_real'])
            writer.writerows(self.data)
        self.get_logger().info(f'💾 Datos guardados en {self.csv_path}')

        # Finalizar nodo
        self.destroy_node()


# -----------------------------------------------------
# MAIN
# -----------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = OpenLoopID()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_and_save()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
