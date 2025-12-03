#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
pid_tuning_node: Pruebas de sintonización
- Modo 'angular' (giro en sitio) o 'lineal' (avance).
- Dos modos de control:
  * pid: w = PID(e) ó v = PID(e)
  * relay: w = ±h ó v = ±h (Åström–Hägglund) para forzar oscilación.
- Log CSV con t, e, cmd, x, y, yaw, ref, sat_flag.
"""

import os, math, time, csv
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile
from turtlebot_scripts.pid_controller import PIDController

CONTROL_DT = 0.033  # 30 Hz

class PIDTuningNode(Node):
    def __init__(self):
        super().__init__('pid_tuning_node')

        # ---- Parámetros ----
        self.declare_parameter('mode', 'angular')      # 'angular' | 'lineal'
        self.declare_parameter('control', 'relay')     # 'relay' | 'pid'
        self.declare_parameter('kp', 1.0)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.0)
        self.declare_parameter('ref_value', 1.5708)    # rad (angular) o m (lineal)
        self.declare_parameter('csv_path', '/ros2_ws/ros2host_ws/src/turtlebot_scripts/turtlebot_scripts/pid_tuning_data_angular.csv')

        # relé
        self.declare_parameter('relay_amp', 0.3)       # h: rad/s o m/s  Angular: 0.25 – 0.35 rad/s Lineal: 0.10 – 0.18 m/s
        self.declare_parameter('deadband', 0.02)       # zona muerta en rad o m Angular: 0.08 – 0.12 rad Lineal: 0.03 – 0.06 m

        # límites (para marcar saturación en el log)
        self.declare_parameter('LIN_MAX', 0.44)
        self.declare_parameter('ANG_MAX', 2.0)

        # ---- Lectura ----
        p = self.get_parameter
        self.mode      = p('mode').value
        self.control   = p('control').value
        self.kp        = float(p('kp').value)
        self.ki        = float(p('ki').value)
        self.kd        = float(p('kd').value)
        self.ref_value = float(p('ref_value').value)
        self.csv_path  = p('csv_path').value
        self.relay_amp = float(p('relay_amp').value)
        self.deadband  = float(p('deadband').value)
        self.LIN_MAX   = float(p('LIN_MAX').value)
        self.ANG_MAX   = float(p('ANG_MAX').value)

        # ---- PID ----
        self.pid = PIDController(self.kp, self.ki, self.kd)

        # ---- Estado ----
        self.pose = None
        self.pose0 = None
        self.last_time = time.time()
        self.t0 = time.time()
        self.last_cmd = 0.0  # para histéresis en relé

        # ---- IO ROS ----
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom_raw', self.odom_cb, QoSProfile(depth=20))
        self.timer = self.create_timer(CONTROL_DT, self.control_loop)

        # ---- CSV ----
        os.makedirs(os.path.dirname(self.csv_path), exist_ok=True)
        self.csv = open(self.csv_path, 'w', newline='')
        self.w = csv.writer(self.csv)
        self.w.writerow(['t','error','cmd','x','y','yaw','ref','sat'])

        self.get_logger().info(f"Modo={self.mode} | Control={self.control} | Ref={self.ref_value:.3f}")
        self.get_logger().info(f"CSV: {self.csv_path}")

    def odom_cb(self, msg):
        self.pose = msg.pose.pose
        if self.pose0 is None:
            self.pose0 = msg.pose.pose

    def yaw_from_quat(self, q):
        siny_cosp = 2 * (q.w*q.z + q.x*q.y)
        cosy_cosp = 1 - 2 * (q.y*q.y + q.z*q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def wrap_pi(self, a):
        return math.atan2(math.sin(a), math.cos(a))

    def control_loop(self):
        if self.pose is None or self.pose0 is None:
            return
        now = time.time()
        dt = max(now - self.last_time, 1e-3)
        self.last_time = now
        t_rel = now - self.t0

        # estado relativo
        x = self.pose.position.x - self.pose0.position.x
        y = self.pose.position.y - self.pose0.position.y
        yaw = self.yaw_from_quat(self.pose.orientation)

        if self.mode == 'angular':
            error = self.wrap_pi(self.ref_value - yaw)
        else:
            dist = math.hypot(x, y)
            error = self.ref_value - dist

        # mando
        if self.control == 'relay':
            if error > self.deadband:
                target_cmd = self.relay_amp
            elif error < -self.deadband:
                target_cmd = -self.relay_amp
            else:
                # mantenerse (histéresis)
                target_cmd = self.last_cmd

            # Suavizado de transición (filtro 1er orden)
            alpha = 0.5  # 0=suave, 1=rápido
            cmd = alpha * target_cmd + (1.0 - alpha) * self.last_cmd
        else:
            cmd = self.pid.compute(error, dt)


        tw = Twist()
        sat = 0
        if self.mode == 'angular':
            # marcar saturación virtual (no recortes duros aquí)
            if abs(cmd) > self.ANG_MAX:
                sat = 1
            tw.angular.z = max(min(cmd,  self.ANG_MAX), -self.ANG_MAX)
        else:
            if abs(cmd) > self.LIN_MAX:
                sat = 1
            tw.linear.x = max(min(cmd, self.LIN_MAX), -self.LIN_MAX)

        self.last_cmd = cmd
        self.cmd_pub.publish(tw)

        # log
        self.w.writerow([t_rel, error, cmd, x, y, yaw, self.ref_value, sat])

        # corta a los 40 s
        if t_rel > 40.0:
            self.finish("Tiempo máximo alcanzado")

    def finish(self, msg):
        self.get_logger().info(f"⏹ {msg}. CSV guardado en {self.csv_path}")
        self.csv.flush()
        self.csv.close()
        # detener robot
        self.cmd_pub.publish(Twist())
        self.destroy_node()

    def destroy_node(self):
        try:
            if not self.csv.closed:
                self.csv.flush()
                self.csv.close()
        except Exception:
            pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = PIDTuningNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.finish("Cancelado por usuario")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
