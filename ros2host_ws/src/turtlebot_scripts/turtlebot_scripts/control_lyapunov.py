#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Nodo: lyapunov_trajectory_node
- Usa el controlador Lyapunov para seguir trayectorias (recta, cuadrada o compuesta).
- Mide odometría, corriente, batería y genera un CSV igual al del PID.
- Publica Path en /robot_path (RViz).
- Incluye seguridad por bumper (pausa inmediata).
"""

import os
import math
import csv
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
)
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
from kobuki_ros_interfaces.msg import SensorState, BumperEvent

from turtlebot_scripts.lyapunov_controller import LyapunovController

# ===================== CONFIGURACIÓN GENERAL =====================
BATTERY_VOLT_PER_UNIT = 0.1
CURRENT_SCALE_A_PER_UNIT = 0.01

LIN_MAX = 0.20
ANG_MAX = 2.0
CONTROL_DT = 0.05
DIST_REACHED = 0.06
BUMPER_BACKOFF_SEC = 20.0

# ===================== GENERADORES DE TRAYECTORIA =====================
def generate_line(length=1.75, points=5):
    xs = [i / (points - 1) * length for i in range(points)]
    ys = [0.0 for _ in range(points)]
    cuts = [len(xs)]
    return xs, ys, cuts

def generate_square(lado=1.75, points_per_side=5):
    xs = [0.0, lado, lado, 0.0, 0.0]
    ys = [0.0, 0.0, lado, lado, 0.0]
    cuts = [1, 2, 3, 4, 5]
    return xs, ys, cuts

def generate_composite(length=1.75, radius=1.75, ppm=2):
    xs, ys, cuts = [], [], []
    n_line = max(int(ppm * length), 10)
    for i in range(n_line):
        xs.append(i / (n_line - 1) * length)
        ys.append(0.0)
        cuts.append(len(xs))
    n_arc = max(int(ppm * (math.pi * radius / 2.0)), 20)
    cx, cy = (length, radius)
    theta0 = -math.pi / 2.0
    theta1 = 0.0
    for i in range(1, n_arc + 1):
        th = theta0 + (theta1 - theta0) * (i / n_arc)
        xs.append(cx + radius * math.cos(th))
        ys.append(cy + radius * math.sin(th))
        cuts.append(len(xs))
    return xs, ys, cuts

# ===================== NODO PRINCIPAL =====================
class LyapunovTrajectoryNode(Node):

    def __init__(self):
        super().__init__('lyapunov_trajectory_node')

        # ---- Parámetros ROS2 ----
        self.declare_parameter('odom_topic', '/odom_raw')
        self.declare_parameter('traj', 'cuadrada')
        self.declare_parameter('lado_m', 1.75)
        self.declare_parameter('radius_m', 1.75)
        self.declare_parameter('points_per_side', 5)
        self.declare_parameter('ppm', 3)

        # ---- Parámetros CSV ----
        self.declare_parameter('csv_name', 'lyapunov')
        self.declare_parameter('csv_dir', '/ros2_ws/ros2host_ws/src/turtlebot_scripts/turtlebot_scripts/')

        self.traj = self.get_parameter('traj').get_parameter_value().string_value
        csv_name = self.get_parameter('csv_name').get_parameter_value().string_value
        csv_dir = self.get_parameter('csv_dir').get_parameter_value().string_value

        fecha_hora = time.strftime("%Y-%d-%m_%H-%M-%S", time.localtime(time.time() - 5 * 3600))
        csv_filename = f"data_{csv_name}_{self.traj}_{fecha_hora}.csv"
        self.csv_path = os.path.join(csv_dir, csv_filename)

        os.makedirs(csv_dir, exist_ok=True)
        self.csv_file = open(self.csv_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'stamp_sec', 'stamp_nsec', 'x', 'y', 'yaw',
            'v_cmd', 'w_cmd', 'rho', 'alpha', 'beta',
            'battery_v', 'i_left_a', 'i_right_a', 'i_tot_a', 'power_w'
        ])
        self.get_logger().info(f"📁 Guardando datos en: {self.csv_path}")

        # ---- Parámetros de trayectoria ----
        self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.lado_m = self.get_parameter('lado_m').get_parameter_value().double_value
        self.radius_m = self.get_parameter('radius_m').get_parameter_value().double_value
        self.points_per_side = self.get_parameter('points_per_side').get_parameter_value().integer_value
        self.ppm = self.get_parameter('ppm').get_parameter_value().integer_value

        # ---- Parámetros del controlador Lyapunov ----
        self.declare_parameter('k_rho', 0.8)
        self.declare_parameter('k_alpha', 1.5)
        self.declare_parameter('k_beta', -0.6)

        k_rho = self.get_parameter('k_rho').get_parameter_value().double_value
        k_alpha = self.get_parameter('k_alpha').get_parameter_value().double_value
        k_beta = self.get_parameter('k_beta').get_parameter_value().double_value
        self.controller = LyapunovController(k_rho, k_alpha, k_beta)

        # ---- Trayectoria ----
        if self.traj == 'recta':
            self.wx_local, self.wy_local, self.cuts = generate_line(self.lado_m, self.points_per_side)
        elif self.traj == 'compuesta':
            self.wx_local, self.wy_local, self.cuts = generate_composite(self.lado_m, self.radius_m, self.ppm)
        else:
            self.wx_local, self.wy_local, self.cuts = generate_square(self.lado_m, self.points_per_side)

        # ---- Estado del robot ----
        self.anchor_done = False
        self.current_idx = 0
        self.pose_rel = (0.0, 0.0, 0.0)
        self.last_t_wall = time.time()
        self.last_battery_v = float('nan')
        self.last_i_left_a = float('nan')
        self.last_i_right_a = float('nan')
        self.paused_by_bumper = False

        # ---- Publicadores / Suscriptores ----
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        path_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        self.path_pub = self.create_publisher(Path, '/robot_path', path_qos)
        self.create_subscription(Odometry, self.odom_topic, self.odom_cb, 10)
        self.create_subscription(SensorState, '/sensors/core', self.core_cb, 10)
        self.create_subscription(BumperEvent, '/events/bumper', self.bumper_cb, 10)

        self.path_msg = Path()
        self.path_msg.header.frame_id = 'odom'
        self.timer = self.create_timer(CONTROL_DT, self.control_loop)
        self.path_timer = self.create_timer(0.5, self.publish_path_again)

        self.get_logger().info(f"Lyapunov listo → traj={self.traj} | k_rho={k_rho}, k_alpha={k_alpha}, k_beta={k_beta}")

    # ===================== CALLBACKS =====================

    def core_cb(self, msg: SensorState):
        self.last_battery_v = float(msg.battery) * BATTERY_VOLT_PER_UNIT
        if len(msg.current) >= 2:
            self.last_i_left_a = float(msg.current[0]) * CURRENT_SCALE_A_PER_UNIT
            self.last_i_right_a = float(msg.current[1]) * CURRENT_SCALE_A_PER_UNIT

    def bumper_cb(self, msg: BumperEvent):
        if msg.state == BumperEvent.PRESSED:
            self.get_logger().warn("⛔ BUMPER PRESIONADO → STOP")
            self.paused_by_bumper = True
            self.stop_robot()

    def odom_cb(self, msg: Odometry):
        pose = msg.pose.pose
        yaw = self.yaw_from_quat(pose.orientation)

        if not self.anchor_done:
            self.x0 = pose.position.x
            self.y0 = pose.position.y
            self.yaw0 = yaw
            c, s = math.cos(self.yaw0), math.sin(self.yaw0)
            self.wx_world = [self.x0 + (x * c - y * s) for x, y in zip(self.wx_local, self.wy_local)]
            self.wy_world = [self.y0 + (x * s + y * c) for x, y in zip(self.wx_local, self.wy_local)]
            self.anchor_done = True
            self.get_logger().info(f"⚓ Trayectoria anclada en ({self.x0:.2f},{self.y0:.2f}) yaw0={self.yaw0:.2f}")

        # Pose relativa
        x_rel = pose.position.x - self.x0
        y_rel = pose.position.y - self.y0
        yaw_rel = self.wrap_pi(yaw - self.yaw0)
        self.pose_rel = (x_rel, y_rel, yaw_rel)

        # Path
        ps = PoseStamped()
        ps.header = msg.header
        ps.header.frame_id = 'odom'
        ps.pose.position.x = x_rel
        ps.pose.position.y = y_rel
        ps.pose.orientation = pose.orientation
        self.path_msg.poses.append(ps)
        if len(self.path_msg.poses) > 4000:
            self.path_msg.poses.pop(0)

    # ===================== CONTROL PRINCIPAL =====================

    def control_loop(self):
        if self.paused_by_bumper or not self.anchor_done:
            return

        x, y, yaw = self.pose_rel
        tx, ty = self.wx_world[self.current_idx], self.wy_world[self.current_idx]

        v, w, rho, alpha, beta = self.controller.compute_control(
            x, y, yaw, tx, ty
        )

        # Saturaciones
        v = max(min(v, LIN_MAX), -LIN_MAX)
        w = max(min(w, ANG_MAX), -ANG_MAX)

        # Publicar velocidades
        tw = Twist()
        tw.linear.x = v
        tw.angular.z = w
        self.cmd_pub.publish(tw)

        # Log de progreso
        if int(time.time() * 10) % 10 == 0:
            self.get_logger().info(f"v={v:.3f} | w={w:.3f} | rho={rho:.3f} | alpha={alpha:.3f}")

        # Guardar CSV
        iTot = self.sum_currents()
        power = self.mul_vi(self.last_battery_v, iTot)
        stamp = self.get_clock().now().to_msg()
        self.csv_writer.writerow([
            stamp.sec, stamp.nanosec, x, y, yaw,
            v, w, rho, alpha, beta,
            self.last_battery_v, self.last_i_left_a, self.last_i_right_a, iTot, power
        ])

        # Avanzar punto
        if rho < DIST_REACHED:
            self.current_idx += 1
            if self.current_idx >= len(self.wx_world):
                self.get_logger().info("🏁 Trayectoria completada (Lyapunov).")
                self.stop_robot()
                self.csv_file.flush()
                return

    # ===================== UTILIDADES =====================
    def publish_path_again(self):
        self.path_msg.header.stamp = self.get_clock().now().to_msg()
        for p in self.path_msg.poses:
            p.header.stamp = self.path_msg.header.stamp
        self.path_pub.publish(self.path_msg)

    def yaw_from_quat(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def wrap_pi(self, a):
        return math.atan2(math.sin(a), math.cos(a))

    def sum_currents(self):
        if math.isnan(self.last_i_left_a) or math.isnan(self.last_i_right_a):
            return float('nan')
        return self.last_i_left_a + self.last_i_right_a

    def mul_vi(self, v, i):
        if math.isnan(v) or math.isnan(i):
            return float('nan')
        return v * i

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

    def destroy_node(self):
        try:
            self.csv_file.flush()
            self.csv_file.close()
        except Exception:
            pass
        super().destroy_node()

# ===================== MAIN =====================
def main(args=None):
    rclpy.init(args=args)
    node = LyapunovTrajectoryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Ctrl+C → Deteniendo robot y cerrando CSV…")
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
