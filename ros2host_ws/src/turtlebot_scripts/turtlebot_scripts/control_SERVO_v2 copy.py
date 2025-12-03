#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Nodo: servo_trajectory_node con PURE PURSUIT + CTE
Servo integrador mejorado:
- Pure Pursuit → waypoint adelantado
- CTE (cross-track error) → corrige error lateral, evita bucles
- Velocidad variable según curvatura
- Feedforward angular
- Anti-windup suave
"""

import os
import math
import csv
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
from kobuki_ros_interfaces.msg import SensorState, BumperEvent
from rclpy.qos import (
    QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
)

# =======================================
# CONTROLADOR SERVO
# =======================================
from turtlebot_scripts.servo_controller import ServoIntegratorController

# =======================================
# CONSTANTES
# =======================================
BATTERY_VOLT_PER_UNIT = 0.12
CURRENT_SCALE_A_PER_UNIT = 0.05

LIN_MAX = 0.18
ANG_MAX = 2.2
ANG_SLEW = 1.0

CONTROL_DT = 0.05

LOOKAHEAD = 0.25     # Pure Pursuit
K_CTE     = 2.0      # Ganancia de corrección lateral (cross-track)


# =======================================
# TRAYECTORIAS
# =======================================
def generate_line(lado=1.75, points_per_side=4):
    return [0.0, lado], [0.0, 0.0], [1, 2]


def generate_square(lado=1.75, points_per_side=4):
    return [0.0, lado, lado, 0.0, 0.0], [0.0, 0.0, lado, lado, 0.0], [1, 2, 3, 4, 5]


# =======================================
# COEFICIENTES DEL SERVO (no se tocan)
# =======================================
# ---- LINEAL ----
A_L = [1.48, -0.5014, 0.02145]
B_L = [2.686, -3.147, 1.047]
C_L = [-2.686, 3.377, -1.277]

# ---- ANGULAR ----
A_A = [0.4157, 0.6872, -0.1029]
B_A = [2.627, -2.137, 0.6006]
C_A = [-2.627, 2.528, -0.9911]


# =====================================================
# NODO PRINCIPAL
# =====================================================
class ServoTrajectoryNode(Node):

    def __init__(self):
        super().__init__("servo_trajectory_node")

        self.start_time = time.time()

        # ---------------------------------------
        # PARÁMETROS
        # ---------------------------------------
        self.declare_parameter("traj", "cuadrada")
        self.declare_parameter("lado_m", 1.75)
        self.declare_parameter("csv_name", "pruebaServo")
        self.declare_parameter(
            "csv_dir",
            "/ros2_ws/ros2host_ws/src/turtlebot_scripts/turtlebot_scripts/pruebas/"
        )

        self.traj = self.get_parameter("traj").value
        self.lado_m = self.get_parameter("lado_m").value

        # ---------------------------------------
        # DISTANCIA DE CAMBIO DE PUNTO
        # ---------------------------------------
        self.DIST_REACHED = 0.017

        # ---------------------------------------
        # CSV
        # ---------------------------------------
        csv_dir = self.get_parameter("csv_dir").value
        csv_base = self.get_parameter("csv_name").value

        fecha = time.strftime(
            "%Y-%d-%m_%H-%M-%S",
            time.localtime(time.time() - 5*3600)
        )
        csv_name = f"data_{csv_base}_{self.traj}_{fecha}.csv"
        self.csv_path = os.path.join(csv_dir, csv_name)

        os.makedirs(csv_dir, exist_ok=True)
        self.csv_file = open(self.csv_path, "w", newline="")
        self.csv_writer = csv.writer(self.csv_file)

        self.csv_writer.writerow([
            "stamp_sec", "stamp_nsec",
            "x", "y", "yaw",
            "v_cmd", "w_cmd", "v_real", "w_real",
            "battery_v", "i_left", "i_right", "i_tot", "power",
            "servo_lin", "servo_ang"
        ])

        self.get_logger().info(f"📁 Guardando datos en: {self.csv_path}")

        # ---------------------------------------
        # CONTROLADORES
        # ---------------------------------------
        self.servo_lin = ServoIntegratorController(A_L, B_L, C_L)
        self.servo_ang = ServoIntegratorController(A_A, B_A, C_A)

        # Anti-windup
        self.servo_lin.set_antiwindup(True, kw=0.45)
        self.servo_ang.set_antiwindup(True, kw=0.40)

        # Feedforward angular
        self.KFF_ANG = 0.35

        # ---------------------------------------
        # TRAYECTORIA
        # ---------------------------------------
        if self.traj == "recta":
            self.wx_local, self.wy_local, _ = generate_line(self.lado_m)
        else:
            self.wx_local, self.wy_local, _ = generate_square(self.lado_m)

        # Estado interno
        self.anchor_done = False
        self.pose_rel = None
        self.last_t = time.time()
        self.current_idx = 0
        self.last_pose = None
        self.last_ang_cmd = 0.0

        # ---------------------------------------
        # ROS PUB/SUB
        # ---------------------------------------
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        path_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        self.path_pub = self.create_publisher(Path, "/robot_path", path_qos)

        self.create_subscription(Odometry, "/odom_raw", self.odom_cb, 10)
        self.create_subscription(SensorState, "/sensors/core", self.core_cb, 10)
        self.create_subscription(BumperEvent, "/events/bumper", self.bumper_cb, 10)

        self.path_msg = Path()
        self.path_msg.header.frame_id = "odom"

        # Timers
        self.timer = self.create_timer(CONTROL_DT, self.control_loop)
        self.path_timer = self.create_timer(0.5, self.publish_path)

        self.get_logger().info("🔧 Nodo Servo listo (PURE PURSUIT + CTE).")

    # ===================================================
    # CALLBACKS
    # ===================================================
    def core_cb(self, msg):
        self.last_batt = msg.battery * BATTERY_VOLT_PER_UNIT
        self.i_left = msg.current[0] * CURRENT_SCALE_A_PER_UNIT
        self.i_right = msg.current[1] * CURRENT_SCALE_A_PER_UNIT

    def bumper_cb(self, msg):
        if msg.state == BumperEvent.PRESSED:
            self.get_logger().warn("⛔ BUMPER PRESIONADO → STOP")
            self.stop_robot()

    def odom_cb(self, msg):

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = self.quat_to_yaw(msg.pose.pose.orientation)

        if not self.anchor_done:
            self.x0 = x
            self.y0 = y
            self.yaw0 = yaw

            c, s = math.cos(yaw), math.sin(yaw)
            self.wx_world = [
                self.x0 + (px*c - py*s)
                for px, py in zip(self.wx_local, self.wy_local)
            ]
            self.wy_world = [
                self.y0 + (px*s + py*c)
                for px, py in zip(self.wx_local, self.wy_local)
            ]

            self.anchor_done = True
            self.get_logger().info("⚓ Trayectoria anclada.")
            return

        xr = x - self.x0
        yr = y - self.y0
        yawr = self.wrap_pi(yaw - self.yaw0)

        self.pose_rel = (xr, yr, yawr)

        ps = PoseStamped()
        ps.header = msg.header
        ps.header.frame_id = "odom"
        ps.pose.position.x = xr
        ps.pose.position.y = yr
        ps.pose.orientation = msg.pose.pose.orientation

        self.path_msg.poses.append(ps)
        if len(self.path_msg.poses) > 2000:
            self.path_msg.poses.pop(0)

    # ===================================================
    # CONTROL LOOP
    # ===================================================
    def control_loop(self):

        if not self.anchor_done or self.pose_rel is None:
            return

        now = time.time()
        dt = max(now - self.last_t, 1e-3)
        self.last_t = now

        x, y, yaw = self.pose_rel

        # ============================================================
        # PURE PURSUIT → SELECCIÓN DEL WAYPOINT DINÁMICO
        # ============================================================
        dx_list = [px - x for px in self.wx_world[self.current_idx:]]
        dy_list = [py - y for py in self.wy_world[self.current_idx:]]
        dist_list = [
            math.hypot(dx_list[i], dy_list[i]) for i in range(len(dx_list))
        ]

        target_idx = None
        for i, d in enumerate(dist_list):
            if d >= LOOKAHEAD:
                target_idx = self.current_idx + i
                break

        if target_idx is None:
            target_idx = self.current_idx

        tx = self.wx_world[target_idx]
        ty = self.wy_world[target_idx]

        dx = tx - x
        dy = ty - y

        dist_err = math.hypot(dx, dy)
        heading_ref = math.atan2(dy, dx)
        yaw_err = self.wrap_pi(heading_ref - yaw)

        # ---- Velocidad real (para log)
        if self.last_pose is not None:
            dxr = x - self.last_pose[0]
            dyr = y - self.last_pose[1]
            dyawr = self.wrap_pi(yaw - self.last_pose[2])

            v_real = math.hypot(dxr, dyr) / dt
            w_real = dyawr / dt
        else:
            v_real = 0.0
            w_real = 0.0

        self.last_pose = (x, y, yaw)

        # ============================================================
        # AJUSTE DE VELOCIDAD SEGÚN CURVATURA
        # ============================================================
        curvature = abs(yaw_err)
        vel_factor = max(0.25, 1.0 - 1.3 * curvature)

        # ============================================================
        # FEEDFORWARD ANGULAR
        # ============================================================
        if hasattr(self, "last_heading_ref"):
            heading_rate = self.wrap_pi(heading_ref - self.last_heading_ref) / dt
        else:
            heading_rate = 0.0

        self.last_heading_ref = heading_ref

        # ============================================================
        # SERVO INTEGRADOR (dist_err, yaw_err)
        # ============================================================
        v_cmd = self.servo_lin.compute(dist_err, 0.0)
        v_cmd *= vel_factor

        w_cmd = self.servo_ang.compute(yaw_err, 0.0)
        w_cmd += self.KFF_ANG * heading_rate

        # ============================================================
        # CTE (CROSS-TRACK ERROR) → CORRECCIÓN LATERAL
        # ============================================================
        # Vector normal a la orientación actual del robot
        nx = -math.sin(yaw)
        ny =  math.cos(yaw)

        # Error lateral: proyección de (tx - x, ty - y) sobre la normal
        cte = dx*nx + dy*ny

        # Corrección lateral sumada al comando angular
        w_cmd += K_CTE * cte

        # ---- Saturaciones
        v_cmd = max(min(v_cmd, LIN_MAX), -LIN_MAX)
        w_cmd = max(min(w_cmd, ANG_MAX), -ANG_MAX)

        # ---- Slew rate
        delta = w_cmd - self.last_ang_cmd
        if delta > ANG_SLEW:
            w_cmd = self.last_ang_cmd + ANG_SLEW
        if delta < -ANG_SLEW:
            w_cmd = self.last_ang_cmd - ANG_SLEW

        self.last_ang_cmd = w_cmd

        # Publicar
        tw = Twist()
        tw.linear.x = v_cmd
        tw.angular.z = w_cmd
        self.cmd_pub.publish(tw)

        # LOG
        stamp = self.get_clock().now().to_msg()
        i_tot = self.i_left + self.i_right
        power = self.last_batt * i_tot

        self.csv_writer.writerow([
            stamp.sec, stamp.nanosec, x, y, yaw,
            v_cmd, w_cmd, v_real, w_real,
            self.last_batt, self.i_left, self.i_right, i_tot, power,
            v_cmd, w_cmd
        ])

        # Cambio de waypoint (cuando estoy cerca del actual)
        if dist_list and dist_list[0] < self.DIST_REACHED:
            self.current_idx += 1

            if self.current_idx >= len(self.wx_world):
                self.get_logger().info("🏁 Trayectoria completada.")
                self.stop_robot()
                return

    # ===================================================
    def publish_path(self):
        self.path_msg.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(self.path_msg)

    # ===================================================
    def stop_robot(self):
        self.cmd_pub.publish(Twist())

    def quat_to_yaw(self, q):
        s = 2*(q.w*q.z + q.x*q.y)
        c = 1 - 2*(q.y*q.y + q.z*q.z)
        return math.atan2(s, c)

    def wrap_pi(self, a):
        return math.atan2(math.sin(a), math.cos(a))


def main(args=None):
    rclpy.init(args=args)
    node = ServoTrajectoryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
