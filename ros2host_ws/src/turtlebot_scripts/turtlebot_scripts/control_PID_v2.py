#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
--------------------------------
Control de seguimiento de trayectoria mediante PID lineal y angular.

Características:
- Compatible con 3 trayectorias: recta, cuadrada y compuesta (recta + arco).
- Utiliza PIDController personalizado mediante compute(error, dt).
- Publica Path en /robot_path para visualización en RViz.
- Registra CSV con odometría, comandos, corriente y potencia.
- Implementa seguridad por bumper (paro inmediato y pausa del controlador).
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

# Controlador PID implementado
from turtlebot_scripts.pid_controller import PIDController


# ===================== CONFIGURACIÓN RÁPIDA =====================
BATTERY_VOLT_PER_UNIT = 0.12           # Escala de voltaje del sensor del Kobuki
CURRENT_SCALE_A_PER_UNIT = 0.01        # Escala de corriente del Kobuki

LIN_MAX = 0.28                          # Velocidad lineal máxima permitida
ANG_MAX = 4.3                           # Velocidad angular máxima
ANG_SLEW = 1.0                          # Límite de cambio por ciclo en w

CONTROL_DT = 0.05                       # Periodo del lazo de control
BUMPER_BACKOFF_SEC = 20.0               # Tiempo de pausa tras bumper


# ====================== GENERADORES DE TRAYECTORIA ======================
def generate_linebk(length=1.75, points=3):
    """Genera una línea recta usando puntos equidistantes."""
    xs = [i / (points - 1) * length for i in range(points)]
    ys = [0.0 for _ in range(points)]
    cuts = [len(xs)]
    return xs, ys, cuts


def generate_line(lado=1.75, points_per_side=4):
    """Genera una trayectoria recta simple entre dos puntos."""
    xs = [0.0, lado]
    ys = [0.0, 0.0]
    cuts = [1, 2]
    return xs, ys, cuts


def generate_square(lado=1.75, points_per_side=4):
    """Genera un cuadrado usando solo las esquinas para un movimiento más fluido."""
    xs = [0.0, lado, lado, 0.0, 0.0]
    ys = [0.0, 0.0, lado, lado, 0.0]
    cuts = [1, 2, 3, 4, 5]
    return xs, ys, cuts


def generate_composite(length=1.75, radius=1.75, points_line=10, ppm_arc=10):
    """
    Trayectoria compuesta: línea recta + arco de 1/4 de circunferencia.
    - points_line controla puntos de la recta.
    - ppm_arc controla puntos por metro del arco.
    """
    xs, ys, cuts = [], [], []

    # Tramo recto
    xs_line = [i / (points_line - 1) * length for i in range(points_line)]
    ys_line = [0.0 for _ in range(points_line)]
    xs.extend(xs_line)
    ys.extend(ys_line)
    cuts.extend(range(1, len(xs_line) + 1))

    # Arco horario 90°
    n_arc = max(int(ppm_arc * (math.pi * radius / 2.0)), 20)
    cx, cy = (length, -radius)

    for i in range(1, n_arc + 1):
        th = (math.pi / 2.0) * (1.0 - i / n_arc)
        xs.append(cx + radius * math.cos(th))
        ys.append(cy + radius * math.sin(th))
        cuts.append(len(xs))

    return xs, ys, cuts


# ============================ NODO PRINCIPAL ============================
class PIDTrajectoryNode(Node):

    def __init__(self):
        super().__init__('pid_trajectory_node')

        # ---------------- Parámetros ROS2 ----------------
        self.start_time = time.time()

        self.declare_parameter('odom_topic', '/odom_raw')
        self.declare_parameter('traj', 'cuadrada')
        self.declare_parameter('lado_m', 1.75)
        self.declare_parameter('radius_m', 1.75)
        self.declare_parameter('points_per_side', 5)
        self.declare_parameter('ppm', 3)

        # Estados de alineación final (solo para cuadrada)
        self.final_align_mode = False
        self.final_stage = 0

        # ---------------- Configuración del archivo CSV ----------------
        self.declare_parameter('csv_name', 'pruebaZiegler')
        self.declare_parameter('csv_dir', '/ros2_ws/ros2host_ws/src/turtlebot_scripts/turtlebot_scripts/pruebas/')

        self.traj = self.get_parameter('traj').get_parameter_value().string_value

        # Tolerancia de llegada según tipo de trayectoria
        if self.traj == "compuesta":
            self.DIST_REACHED = 0.06        # Mayor tolerancia en curva
        else:
            self.DIST_REACHED = 0.015       # Tolerancia fina en recta/cuadrada

        csv_name = self.get_param_string('csv_name')
        csv_dir = self.get_param_string('csv_dir')

        # Nombre dinámico del archivo CSV
        fecha_hora = time.strftime("%Y-%d-%m_%H-%M-%S", time.localtime(time.time() - 5 * 3600))
        csv_filename = f"data_{csv_name}_{self.traj}_{fecha_hora}.csv"
        self.csv_path = os.path.join(csv_dir, csv_filename)

        # Creación del archivo CSV
        os.makedirs(csv_dir, exist_ok=True)
        self.csv_file = open(self.csv_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'stamp_sec', 'stamp_nsec', 'x', 'y', 'yaw',
            'v_cmd', 'w_cmd', 'v_real', 'w_real',
            'battery_v', 'i_left_a', 'i_right_a', 'i_tot_a', 'power_w',
            'pid_lin', 'pid_ang'
        ])

        self.get_logger().info(f"Registro iniciado en: {self.csv_path}")

        # ---------------- Lectura del resto de parámetros ----------------
        self.odom_topic = self.get_param_string('odom_topic')
        self.lado_m = self.get_param_double('lado_m')
        self.radius_m = self.get_param_double('radius_m')
        self.points_per_side = self.get_param_int('points_per_side')
        self.ppm = self.get_param_int('ppm')

        # ---------------- Ganancias PID  COHEN–COON ----------------
        self.declare_parameter('kp_lin', 3.4246)
        self.declare_parameter('ki_lin', 16.5569)
        self.declare_parameter('kd_lin', 0.1144)

        self.declare_parameter('kp_ang', 3.9273)
        self.declare_parameter('ki_ang', 7.1554)
        self.declare_parameter('kd_ang', 0.3450)

                """
        # --- Controlador LINEAL (Ziegler–Nichols – PID) ---
        self.declare_parameter('kp_lin', 2.8254)    # Ganancia proporcional
        self.declare_parameter('ki_lin', 14.1268)   # Ganancia integral
        self.declare_parameter('kd_lin', 0.1413)    # Ganancia derivativa
        
        # --- Controlador ANGULAR (Ziegler–Nichols – PID) ---
        self.declare_parameter('kp_ang', 3.2773)    # Ganancia proporcional
        self.declare_parameter('ki_ang', 6.3026)    # Ganancia integral
        self.declare_parameter('kd_ang', 0.4261)    # Ganancia derivativa
        """

        kp_lin = self.get_param_double('kp_lin')
        ki_lin = self.get_param_double('ki_lin')
        kd_lin = self.get_param_double('kd_lin')

        kp_ang = self.get_param_double('kp_ang')
        ki_ang = self.get_param_double('ki_ang')
        kd_ang = self.get_param_double('kd_ang')


        # Publicadores y suscriptores
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

        # Instancias PID
        self.lin_pid = PIDController(kp_lin, ki_lin, kd_lin)
        self.ang_pid = PIDController(kp_ang, ki_ang, kd_ang)

        # Selección de trayectoria
        if self.traj == 'recta':
            self.wx_local, self.wy_local, self.cuts = generate_line(self.lado_m, self.points_per_side)
        elif self.traj == 'compuesta':
            self.wx_local, self.wy_local, self.cuts = generate_composite(self.lado_m, self.radius_m, self.ppm)
        else:
            self.wx_local, self.wy_local, self.cuts = generate_square(self.lado_m, self.points_per_side)

        # ---------------- Variables del nodo ----------------
        self.wx_world = None
        self.wy_world = None
        self.anchor_done = False
        self.pose = None
        self.current_idx = 0
        self.current_seg = 1
        self.aligning = True
        self.corner_mode = False
        self.corner_yaw_target = None
        self.last_ang_cmd = 0.0
        self.last_t_wall = time.time()

        # Sensores
        self.paused_by_bumper = False
        self.bumper_last_pressed = False
        self.resume_at = 0.0

        self.last_battery_v = float('nan')
        self.last_i_left_a = float('nan')
        self.last_i_right_a = float('nan')

        # Path
        self.odom_frame = 'odom'
        self.path_msg = Path()
        self.path_msg.header.frame_id = self.odom_frame
        self.max_path_len = 4000

        # Timers del nodo
        self.timer = self.create_timer(CONTROL_DT, self.control_loop)
        self.path_timer = self.create_timer(0.5, self.publish_path_again)

        # Logs iniciales
        self.get_logger().info(
            f"PID Trajectory configurado | traj={self.traj} | L={self.lado_m} | points={self.points_per_side}"
        )
        self.get_logger().info(
            f"PID lin=({kp_lin},{ki_lin},{kd_lin}) | PID ang=({kp_ang},{ki_ang},{kd_ang})"
        )

    # ------------------- Callbacks -------------------

    def core_cb(self, msg: SensorState):
        """Lee batería y corrientes desde el Kobuki."""
        self.last_battery_v = float(msg.battery) * BATTERY_VOLT_PER_UNIT
        if len(msg.current) >= 2:
            self.last_i_left_a = float(msg.current[0]) * CURRENT_SCALE_A_PER_UNIT
            self.last_i_right_a = float(msg.current[1]) * CURRENT_SCALE_A_PER_UNIT

    def bumper_cb(self, msg: BumperEvent):
        """Activa pausa del nodo si el bumper detecta colisión."""
        pressed = (msg.state == BumperEvent.PRESSED)
        if pressed and not self.bumper_last_pressed:
            self.get_logger().warn("BUMPER ACTIVADO → Movimiento detenido")
            self.paused_by_bumper = True
            self.stop_robot()
        self.bumper_last_pressed = pressed

    def odom_cb(self, msg: Odometry):
        """
        Callback de odometría.
        - Ancla la trayectoria al marco inicial.
        - Calcula pose relativa.
        - Construye path para RViz.
        """
        self.odom_frame = msg.header.frame_id or self.odom_frame

        if not self.anchor_done:
            self.x0 = msg.pose.pose.position.x
            self.y0 = msg.pose.pose.position.y
            self.yaw0 = self.yaw_from_quat(msg.pose.pose.orientation)

            # Transformación a marco global
            c, s = math.cos(self.yaw0), math.sin(self.yaw0)
            self.wx_world = [self.x0 + (x * c - y * s) for x, y in zip(self.wx_local, self.wy_local)]
            self.wy_world = [self.y0 + (x * s + y * c) for x, y in zip(self.wx_local, self.wy_local)]
            self.anchor_done = True

            self.get_logger().info(
                f"Trayectoria anclada en ({self.x0:.2f},{self.y0:.2f}) yaw0={self.yaw0:.2f}"
            )

        # Pose relativa
        pose = msg.pose.pose
        yaw = self.yaw_from_quat(pose.orientation)

        x_rel = pose.position.x - self.x0
        y_rel = pose.position.y - self.y0
        yaw_rel = self.wrap_pi(yaw - self.yaw0)
        self.pose_rel = (x_rel, y_rel, yaw_rel)

        # Path relativo para RViz
        ps = PoseStamped()
        ps.header = msg.header
        ps.header.frame_id = 'odom'
        ps.pose.position.x = x_rel
        ps.pose.position.y = y_rel
        ps.pose.position.z = 0.0
        ps.pose.orientation = pose.orientation

        self.path_msg.poses.append(ps)
        if len(self.path_msg.poses) > self.max_path_len:
            self.path_msg.poses.pop(0)

    # ------------------- Lazo de control principal -------------------

    def control_loop(self):
        """Ejecución periódica del lazo PID de posición."""
        if self.paused_by_bumper or not self.anchor_done or not hasattr(self, 'pose_rel'):
            return

        # -------- Alineación final (solo cuadrada) --------
        # Mantengo tu lógica exactamente igual.
        # Mejoro comentarios para estilo profesional.
        if self.final_align_mode:
            x, y, yaw = self.get_pose_tuple()

            # Etapa 0: retroceso inicial
            if self.final_stage == 0:
                dist_from_origin = math.hypot(x, y)
                if dist_from_origin > 0.04:
                    self.get_logger().info("Retroceso completado. Iniciando alineación angular.")
                    self.final_stage = 1
                    time.sleep(0.2)
                    return

                tw = Twist()
                tw.linear.x = -0.09
                self.cmd_pub.publish(tw)
                return

            # Etapa 1: orientación hacia el origen
            if self.final_stage == 1:
                dx = -x
                dy = -y
                target_heading = math.atan2(dy, dx)
                yaw_err = self.wrap_pi(target_heading - yaw)

                tw = Twist()
                tw.angular.z = 0.80 * max(-1, min(1, yaw_err / 1.0))
                self.cmd_pub.publish(tw)

                if abs(yaw_err) < 0.10:
                    self.get_logger().info("Orientación completada.")
                    self.final_stage = 2
                    time.sleep(0.3)
                return

            # Etapa 2: avance final al origen
            if self.final_stage == 2:
                dist = math.hypot(x, y)
                if dist < 0.03:
                    self.get_logger().info("Llegada al origen confirmada.")
                    self.final_stage = 3
                    time.sleep(0.2)
                    return

                tw = Twist()
                tw.linear.x = 0.10
                self.cmd_pub.publish(tw)
                return

            # Etapa 3: orientación final a yaw inicial
            if self.final_stage == 3:
                yaw_err = self.wrap_pi(0.0 - yaw)

                tw = Twist()
                tw.angular.z = 0.40 * max(-1, min(1, yaw_err / 0.5))
                self.cmd_pub.publish(tw)

                if abs(yaw_err) < 0.10:
                    self.get_logger().info("Alineación final completada.")
                    self.stop_robot()
                    self.final_align_mode = False
                return

        # -------- Validación de sensores --------
        if math.isnan(self.last_battery_v):
            self.last_battery_v = 12.0

        now = time.time()
        dt = max(now - getattr(self, 'last_t_wall', now), 1e-3)
        self.last_t_wall = now

        # -------- Cálculo de error de referencia --------
        tx = self.wx_world[self.current_idx]
        ty = self.wy_world[self.current_idx]
        x, y, yaw = self.get_pose_tuple()

        dx = tx - x
        dy = ty - y
        dist_err = math.hypot(dx, dy)
        heading_ref = math.atan2(dy, dx)
        yaw_err = self.wrap_pi(heading_ref - yaw)

        # -------- Modo de alineación inicial --------
        if self.current_idx == 0 and abs(yaw_err) < 0.3:
            self.aligning = False

        if self.aligning:
            v = max(0.03, 0.18 * max(0.0, (1.0 - abs(yaw_err))))
            w = self.ang_pid.compute(yaw_err, dt)
        else:
            v = self.lin_pid.compute(dist_err, dt)
            w = self.ang_pid.compute(yaw_err, dt)

        # -------- Dead-band para vencer fricción --------
        vmin_base = 0.12
        vmin = vmin_base * (12.0 / max(self.last_battery_v, 12.0))

        if dist_err > 0.20:
            if abs(yaw_err) < 0.25 and abs(w) < 0.6:
                if 0 < v < vmin: v = vmin
                if -vmin < v < 0: v = -vmin

        # -------- Limitador dinámico por batería --------
        vmax = LIN_MAX * (self.last_battery_v / 12.0) if self.last_battery_v < 12.0 else LIN_MAX

        # -------- Rampa de frenado (solo cuadrada) --------
        if self.traj == "cuadrada":
            RAMP_DIST = 0.10
            MIN_FACTOR = 0.8

            if dist_err < RAMP_DIST:
                ratio = dist_err / RAMP_DIST
                factor = MIN_FACTOR + (1 - MIN_FACTOR) * (ratio ** 2)
                vmax = vmax * factor

        v = max(min(v, vmax), -vmax)

        # -------- Suavizado de aceleración --------
        max_accel = 0.5
        if not hasattr(self, 'last_v_cmd'):
            self.last_v_cmd = 0.0

        v_step = max_accel * dt
        if v > self.last_v_cmd + v_step:
            v = self.last_v_cmd + v_step
        elif v < self.last_v_cmd - v_step:
            v = self.last_v_cmd - v_step

        self.last_v_cmd = v

        # -------- Límite angular y suavizado --------
        w = max(min(w, ANG_MAX), -ANG_MAX)
        w = self.apply_slew(w, self.last_ang_cmd, ANG_SLEW)
        self.last_ang_cmd = w

        # -------- Publicación de comandos --------
        tw = Twist()
        tw.linear.x = v
        tw.angular.z = w
        self.cmd_pub.publish(tw)

        # -------- Estimación de velocidades reales --------
        if hasattr(self, 'last_pose'):
            dx = x - self.last_pose[0]
            dy = y - self.last_pose[1]
            dyaw = self.wrap_pi(yaw - self.last_pose[2])
            dt_vel = max(dt, 1e-3)

            v_real = math.hypot(dx, dy) / dt_vel
            w_real = dyaw / dt_vel
        else:
            v_real = 0.0
            w_real = 0.0

        self.last_pose = (x, y, yaw)

        # -------- Registro en CSV --------
        iTot = self.sum_currents()
        power = self.mul_vi(self.last_battery_v, iTot)
        stamp = self.get_clock().now().to_msg()

        pid_lin_str = f"({self.lin_pid.kp:.3f},{self.lin_pid.ki:.3f},{self.lin_pid.kd:.3f})"
        pid_ang_str = f"({self.ang_pid.kp:.3f},{self.ang_pid.ki:.3f},{self.ang_pid.kd:.3f})"

        self.csv_writer.writerow([
            stamp.sec, stamp.nanosec, x, y, yaw,
            v + 0.08, w, v_real + 0.08, w_real,
            self.last_battery_v,
            self.last_i_left_a, self.last_i_right_a, iTot, power,
            pid_lin_str, pid_ang_str
        ])

        # -------- Avance al siguiente punto --------
        if dist_err < self.DIST_REACHED:
            self.current_idx += 1

            if self.current_idx >= len(self.wx_world):

                if self.traj == "cuadrada" and not self.final_align_mode:
                    self.get_logger().info("Trayectoria completada. Iniciando alineación final.")
                    self.stop_robot()
                    time.sleep(1.0)
                    self.final_align_mode = True
                    self.final_stage = 0
                    return

                self.get_logger().info("Proceso completado.")
                self.stop_robot()
                self.csv_file.flush()
                return

    # ------------------- Publicación de Path -------------------
    def publish_path_again(self):
        """Re-publica el path para mantener persistencia en RViz."""
        if self.path_msg is None:
            return
        self.path_msg.header.stamp = self.get_clock().now().to_msg()
        for p in self.path_msg.poses:
            p.header.stamp = self.path_msg.header.stamp
        self.path_pub.publish(self.path_msg)

    # ------------------- Utilidades -------------------
    def yaw_from_quat(self, q):
        """Convierte orientación en cuaternión a yaw."""
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def wrap_pi(self, a):
        """Ajusta el ángulo al rango [-π, π]."""
        return math.atan2(math.sin(a), math.cos(a))

    def apply_slew(self, new_val, last_val, max_step):
        """Limita la variación entre ciclos para suavizar comandos."""
        delta = new_val - last_val
        if delta > max_step:
            return last_val + max_step
        elif delta < -max_step:
            return last_val - max_step
        return new_val

    def get_pose_tuple(self):
        """Retorna la pose relativa actual (x, y, yaw)."""
        if hasattr(self, 'pose_rel'):
            return self.pose_rel
        return (0.0, 0.0, 0.0)

    def sum_currents(self):
        """Suma las corrientes izquierda y derecha del Kobuki."""
        if math.isnan(self.last_i_left_a) or math.isnan(self.last_i_right_a):
            return float('nan')
        return self.last_i_left_a + self.last_i_right_a

    def mul_vi(self, v, i):
        """Calcula potencia eléctrica instantánea."""
        if math.isnan(v) or math.isnan(i):
            return float('nan')
        return v * i

    def stop_robot(self):
        """Publica /cmd_vel en cero para detener el robot."""
        self.cmd_pub.publish(Twist())

    def destroy_node(self):
        """Cierra archivo CSV y destruye el nodo."""
        try:
            if hasattr(self, 'csv_file') and self.csv_file:
                self.csv_file.flush()
                self.csv_file.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    """Inicialización estándar de un nodo ROS2."""
    rclpy.init(args=args)
    node = PIDTrajectoryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Ctrl+C → Deteniendo robot y cerrando CSV.")
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
