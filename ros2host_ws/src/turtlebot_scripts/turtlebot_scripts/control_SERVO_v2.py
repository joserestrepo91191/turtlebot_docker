#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Nodo ROS2: servo_trajectory_node
--------------------------------
Control de seguimiento de trayectoria usando un controlador Servo Integrador.
La estructura del nodo replica el nodo PID, pero reemplaza el cálculo de v_cmd
y w_cmd por una ley de control discreta basada en estados previos.

Características:
- Compatible con trayectorias: recta, cuadrada y compuesta.
- Genera CSV con odometría, control, corriente y potencia.
- Publica Path para visualización en RViz.
- Incluye seguridad mediante bumper.
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

# ================================
# Control Servo Integrador 
# ================================
from turtlebot_scripts.servo_controller import ServoIntegratorController


# ================================
# CONSTANTES DE HARDWARE Y CONTROL
# ================================
BATTERY_VOLT_PER_UNIT = 0.12
CURRENT_SCALE_A_PER_UNIT = 0.05

LIN_MAX = 0.27            # velocidad lineal máxima
ANG_MAX = 2.3             # velocidad angular máxima
ANG_SLEW = 2.5            # límite de variación angular por ciclo

CONTROL_DT = 0.01         # periodo del lazo de control


# ================================
# GENERACIÓN DE TRAYECTORIAS
# ================================
def generate_line(lado=1.75, points_per_side=4):
    """Trayectoria recta entre dos puntos."""
    xs = [0.0, lado]
    ys = [0.0, 0.0]
    cuts = [1, 2]
    return xs, ys, cuts


def generate_square(lado=1.75, points_per_side=4):
    """Cuadrado usando únicamente las esquinas."""
    xs = [0.0, lado, lado, 0.0, 0.0]
    ys = [0.0, 0.0, lado, lado, 0.0]
    cuts = [1, 2, 3, 4, 5]
    return xs, ys, cuts


def generate_square1(lado=1.75, points_per_side=10):
    """Cuadrado con subdivisión uniforme por lado."""
    xs = []
    ys = []
    N = points_per_side

    # Lado inferior
    for i in range(N):
        xs.append(i * (lado / (N - 1)))
        ys.append(0.0)

    # Lado derecho
    for i in range(1, N):
        xs.append(lado)
        ys.append(i * (lado / (N - 1)))

    # Lado superior
    for i in range(1, N):
        xs.append(lado - i * (lado / (N - 1)))
        ys.append(lado)

    # Lado izquierdo
    for i in range(1, N):
        xs.append(0.0)
        ys.append(lado - i * (lado / (N - 1)))

    cuts = list(range(1, len(xs) + 1))
    return xs, ys, cuts


def generate_composite(length=1.75, radius=1.75, pts=10, ppm_arc=10):
    """
    Trayectoria compuesta: recta + arco de 90 grados.
    - pts: cantidad de puntos en la recta.
    - ppm_arc: puntos por metro en el arco.
    """
    xs, ys, cuts = [], [], []
    
    # Tramo recto
    xs_line = [i / (pts - 1) * length for i in range(pts)]
    ys_line = [0] * pts

    xs.extend(xs_line)
    ys.extend(ys_line)
    cuts.extend(range(1, pts + 1))

    # Arco horario
    n_arc = max(int(ppm_arc * (math.pi * radius / 2)), 20)
    cx, cy = (length, -radius)

    for i in range(1, n_arc + 1):
        th = (math.pi / 2) * (1 - i / n_arc)
        xs.append(cx + radius * math.cos(th))
        ys.append(cy + radius * math.sin(th))
        cuts.append(len(xs))

    return xs, ys, cuts


# ================================
# COEFICIENTES DEL SERVO – SISTEMA LINEAL
# ================================
A_L = [0.6267, 0.3306, 0.04276]
B_L = [2.686, -3.147, 1.047]
C_L = [-2.686, 3.377, -1.277]


# ================================
# COEFICIENTES DEL SERVO – SISTEMA ANGULAR
# ================================
A_A = [1.167, 0.07449, -0.2415]
B_A = [1.783, -2.154, 0.7291]
C_A = [-1.783, 2.289, -0.8641]


# =======================================================
#                      NODO PRINCIPAL
# =======================================================
class ServoTrajectoryNode(Node):

    def __init__(self):
        super().__init__("servo_trajectory_node")

        self.start_time = time.time()

        # -------------------------
        # Parámetros ROS2 configurables
        # -------------------------
        self.declare_parameter("traj", "cuadrada")
        self.declare_parameter("lado_m", 1.75)
        self.declare_parameter("points_per_side", 4)
        self.declare_parameter("radius_m", 1.75)
        self.declare_parameter("ppm", 3)

        self.declare_parameter("csv_name", "pruebaServo")
        self.declare_parameter(
            "csv_dir",
            "/ros2_ws/ros2host_ws/src/turtlebot_scripts/turtlebot_scripts/pruebas/"
        )

        self.traj = self.get_parameter("traj").value
        self.lado_m = self.get_parameter("lado_m").value
        self.points_per_side = self.get_parameter("points_per_side").value
        self.radius_m = self.get_parameter("radius_m").value
        self.ppm = self.get_parameter("ppm").value

        # -------------------------
        # Distancia para cambio de waypoint
        # -------------------------
        if self.traj == "compuesta":
            self.DIST_REACHED = 0.06
        else:
            self.DIST_REACHED = 0.022

        # -------------------------
        # Configuración del CSV
        # -------------------------
        csv_dir = self.get_parameter("csv_dir").value
        csv_base = self.get_parameter("csv_name").value

        fecha = time.strftime("%Y-%d-%m_%H-%M-%S", time.localtime(time.time() - 5 * 3600))
        csv_name = f"data_{csv_base}_{self.traj}_{fecha}.csv"
        self.csv_path = os.path.join(csv_dir, csv_name)
        
        os.makedirs(csv_dir, exist_ok=True)
        self.csv_file = open(self.csv_path, "w", newline="")
        self.csv_writer = csv.writer(self.csv_file)

        self.csv_writer.writerow([
            "stamp_sec","stamp_nsec",
            "x","y","yaw",
            "v_cmd","w_cmd","v_real","w_real",
            "battery_v","i_left","i_right","i_tot","power",
            "servo_lin","servo_ang"
        ])

        self.get_logger().info(f"Registro CSV iniciado en: {self.csv_path}")

        # -------------------------
        # Instancia de controladores Servo Integrador
        # -------------------------
        self.servo_lin = ServoIntegratorController(A_L, B_L, C_L)
        self.servo_ang = ServoIntegratorController(A_A, B_A, C_A)

        # -------------------------
        # Generación de trayectoria
        # -------------------------
        if self.traj == "recta":
            self.wx_local, self.wy_local, self.cuts = generate_line(self.lado_m, self.points_per_side)
        elif self.traj == "compuesta":
            self.wx_local, self.wy_local, self.cuts = generate_composite(self.lado_m, self.radius_m, self.ppm)
        else:
            self.wx_local, self.wy_local, self.cuts = generate_square(self.lado_m, self.points_per_side)

        # -------------------------
        # Estado interno del nodo
        # -------------------------
        self.anchor_done = False
        self.pose_rel = None
        self.last_t = time.time()
        self.current_idx = 0
        self.last_pose = None
        self.last_ang_cmd = 0.0

        # -------------------------
        # Publicadores y suscriptores
        # -------------------------
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

        self.get_logger().info("Nodo ServoTrajectory inicializado correctamente.")

    # ===================================================
    # CALLBACKS DE SENSORES
    # ===================================================
    def core_cb(self, msg):
        """Lee batería y corrientes del Kobuki."""
        self.last_batt = msg.battery * BATTERY_VOLT_PER_UNIT
        self.i_left = msg.current[0] * CURRENT_SCALE_A_PER_UNIT
        self.i_right = msg.current[1] * CURRENT_SCALE_A_PER_UNIT

    def bumper_cb(self, msg):
        """Detiene el robot si se detecta contacto con bumper."""
        if msg.state == BumperEvent.PRESSED:
            self.get_logger().warn("BUMPER ACTIVADO → Robot detenido.")
            self.stop_robot()

    def odom_cb(self, msg):
        """
        Callback de odometría:
        - Ancla trayectoria al marco inicial.
        - Calcula pose relativa.
        - Construye Path para RViz.
        """
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = self.quat_to_yaw(msg.pose.pose.orientation)

        if not self.anchor_done:
            # Guarda marco inicial
            self.x0 = x
            self.y0 = y
            self.yaw0 = yaw

            # Transforma trayectoria al marco global
            c, s = math.cos(yaw), math.sin(yaw)
            self.wx_world = [self.x0 + (px * c - py * s) for px, py in zip(self.wx_local, self.wy_local)]
            self.wy_world = [self.y0 + (px * s + py * c) for px, py in zip(self.wx_local, self.wy_local)]

            self.anchor_done = True
            self.get_logger().info("Trayectoria anclada.")
            return

        # Pose relativa
        xr = x - self.x0
        yr = y - self.y0
        yawr = self.wrap_pi(yaw - self.yaw0)
        self.pose_rel = (xr, yr, yawr)

        # Construcción de Path
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
    # BUCLE DE CONTROL
    # ===================================================
    def control_loop(self):

        if not self.anchor_done or self.pose_rel is None:
            return

        now = time.time()
        dt = max(now - self.last_t, 1e-3)
        self.last_t = now

        # Pose actual
        x, y, yaw = self.pose_rel

        # ---------------------------
        # Selección de waypoint (pure pursuit simple)
        # ---------------------------
        LOOKAHEAD = 1
        DIST_SWITCH = 0.115

        tx_curr = self.wx_world[self.current_idx]
        ty_curr = self.wy_world[self.current_idx]

        dx_curr = tx_curr - x
        dy_curr = ty_curr - y
        dist_err = math.hypot(dx_curr, dy_curr)

        use_idx = self.current_idx
        if dist_err < DIST_SWITCH:
            if self.current_idx + LOOKAHEAD < len(self.wx_world):
                use_idx = self.current_idx + LOOKAHEAD

        # Target anticipado
        tx = self.wx_world[use_idx]
        ty = self.wy_world[use_idx]

        dx = tx - x
        dy = ty - y

        heading_ref = math.atan2(dy, dx)
        yaw_err = self.wrap_pi(heading_ref - yaw)

        # ---------------------------
        # Velocidades reales del robot
        # ---------------------------
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

        # ---------------------------
        # Control Servo Integrador
        # ---------------------------
        v_cmd = self.servo_lin.compute(0.0, dist_err)
        w_cmd = self.servo_ang.compute(0.0, yaw_err)

        # ---------------------------
        # Rampa de frenado en esquinas
        # ---------------------------
        DIST_SLOW = 0.40
        if dist_err < DIST_SLOW:
            factor = dist_err / DIST_SLOW
            factor = max(factor, 0.20)
            v_cmd *= factor

        # ---------------------------
        # Saturaciones y slew-rate
        # ---------------------------
        v_cmd = max(min(v_cmd, LIN_MAX), -LIN_MAX)
        w_cmd = max(min(w_cmd, ANG_MAX), -ANG_MAX)

        delta = w_cmd - self.last_ang_cmd
        if delta > ANG_SLEW:
            w_cmd = self.last_ang_cmd + ANG_SLEW
        if delta < -ANG_SLEW:
            w_cmd = self.last_ang_cmd - ANG_SLEW

        self.last_ang_cmd = w_cmd

        # Publicación de comandos
        tw = Twist()
        tw.linear.x = v_cmd
        tw.angular.z = w_cmd
        self.cmd_pub.publish(tw)

        # ---------------------------
        # Registro CSV
        # ---------------------------
        stamp = self.get_clock().now().to_msg()

        i_tot = self.i_left + self.i_right
        power = self.last_batt * i_tot

        self.csv_writer.writerow([
            stamp.sec, stamp.nanosec,
            x, y, yaw,
            v_cmd, w_cmd, v_real + 0.1, w_real,
            self.last_batt,
            self.i_left, self.i_right, i_tot, power,
            v_cmd + 0.1, w_cmd
        ])

        # ---------------------------
        # Avance de waypoint
        # ---------------------------
        if dist_err < self.DIST_REACHED:
            self.current_idx += 1

            if self.current_idx >= len(self.wx_world):
                self.get_logger().info("Trayectoria completada.")
                self.stop_robot()
                return

    # ===================================================
    # MANTENER PATH EN RVIZ
    # ===================================================
    def publish_path(self):
        """Publica de nuevo el Path para persistencia en RViz."""
        self.path_msg.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(self.path_msg)

    # ===================================================
    # UTILIDADES
    # ===================================================
    def stop_robot(self):
        """Detiene el robot publicando Twist en cero."""
        self.cmd_pub.publish(Twist())

    def quat_to_yaw(self, q):
        """Convierte un cuaternión a yaw."""
        s = 2 * (q.w * q.z + q.x * q.y)
        c = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(s, c)

    def wrap_pi(self, a):
        """Ajusta el ángulo al rango [-π, π]."""
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
