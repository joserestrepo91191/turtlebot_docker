#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
from kobuki_ros_interfaces.msg import BumperEvent

import math
import time
import csv
import numpy as np

from turtlebot_scripts.pid_controller import PIDController


class TrajectoryGenerator:
    """
    Genera un cuadrado local: (0,0)->(1.75,0)->(1.75,1.75)->(0,1.75)->(0,0)
    Interpola puntos y yaw; además expone los cortes de cada lado (side_breaks).
    """
    def __init__(self, N=5):
        self.div = round(N / 4)  # resolución base por lado
        self.pointX = [0, 1.75, 1.75, 0, 0]
        self.pointY = [0, 0, 1.75, 1.75, 0]
        self.pointYaw = np.radians([0.1, 90, 179, -90, -0.1])

        self.px, self.py, self.pyaw = [], [], []
        self.side_breaks = []  # índices de fin de cada lado (exclusivo)
        self.generate_trajectory()

    def generate_trajectory(self):
        total = 0
        for p in range(len(self.pointX) - 1):
            dx = self.pointX[p + 1] - self.pointX[p]
            dy = self.pointY[p + 1] - self.pointY[p]
            distance = float(np.hypot(dx, dy))
            local_div = max(int(self.div * distance / 0.1), 10)

            # interp lineal en XY
            self.px.extend(np.linspace(self.pointX[p], self.pointX[p + 1], local_div))
            self.py.extend(np.linspace(self.pointY[p], self.pointY[p + 1], local_div))

            # interp circular en yaw (envoltura ±π)
            yaw_start = self.pointYaw[p]
            yaw_end = self.pointYaw[p + 1]
            if yaw_end - yaw_start > math.pi:
                yaw_end -= 2 * math.pi
            elif yaw_end - yaw_start < -math.pi:
                yaw_end += 2 * math.pi
            self.pyaw.extend(np.linspace(yaw_start, yaw_end, local_div))

            total += local_div
            if p < 4:
                self.side_breaks.append(total)

    def get_trajectory(self):
        return self.px, self.py, self.pyaw, self.side_breaks


class TurtlebotTrajectoryFollower(Node):
    def __init__(self):
        super().__init__('turtlebot_pid_trajectory_follower')

        # Pubs/Subs
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(Odometry, '/odom_raw', self.odom_callback, 10)
        self.path_pub = self.create_publisher(Path, '/robot_path', 10)
        self.bumper_sub = self.create_subscription(BumperEvent, '/events/bumper', self.bumper_callback, 10)

        # PID (recta más “decidida”; angular menos agresivo)
        self.linear_pid  = PIDController(2.0, 0.00, 0.05)
        self.angular_pid = PIDController(2.0, 0.00, 0.80)

        # Límites
        self.LIN_MAX = 0.22          # m/s
        self.ANG_MAX = 1.30          # rad/s

        # Histéresis alineación
        self.YAW_ENTER_ALIGN = 0.35  # entrar a giro en sitio
        self.YAW_EXIT_ALIGN  = 0.12  # salir de giro en sitio
        self._aligning = True

        # Rampa angular (slew-rate)
        self._last_ang = 0.0
        self.ANG_SLEW = 0.25  # rad/s por ciclo (~7.6 rad/s^2 con dt ≈ 0.033)

        # Umbrales distancia
        self.DIST_REACHED = 0.05  # m

        # Comando
        self.command = Twist()

        # Trayectoria base local (se ancla con primer odom)
        traj = TrajectoryGenerator(N=5)
        self.waypoints_x, self.waypoints_y, self.waypoints_yaw, self.side_breaks = traj.get_trajectory()

        self.current_index = 0
        self.current_side = 1  # 1..4
        self.pose = None
        self.last_time = time.time()

        # Modo “giro de esquina” robusto
        self._corner_mode = False
        self._target_yaw_corner = None

        # Path (RViz)
        self.path = Path()
        self.odom_frame = None
        self.MAX_POSES = 3000

        # Estados
        self.execution_stopped = False
        self.anchor_set = False

        # Timer de control ~30 Hz
        self.timer = self.create_timer(0.033, self.control_loop)

        # Rate-limit de logs numéricos
        self._last_log_t = 0.0
        self._log_dt = 0.5  # s

        self.get_logger().info('Nodo PID trayectoria cuadrada listo. Esperando odometría...')

    # ===================== Callbacks =====================

    def odom_callback(self, msg: Odometry):
        current_pose = PoseStamped()
        current_pose.header = msg.header
        current_pose.pose = msg.pose.pose

        if not self.anchor_set:
            x0 = current_pose.pose.position.x
            y0 = current_pose.pose.position.y
            yaw0 = self.get_yaw_from_quaternion(current_pose.pose.orientation)

            cos0, sin0 = math.cos(yaw0), math.sin(yaw0)
            x_rot, y_rot = [], []
            for x, y in zip(self.waypoints_x, self.waypoints_y):
                xr = x * cos0 - y * sin0 + x0
                yr = x * sin0 + y * cos0 + y0
                x_rot.append(xr)
                y_rot.append(yr)
            self.waypoints_x, self.waypoints_y = x_rot, y_rot
            self.anchor_set = True

            self.get_logger().info(
                f"⚓️ Trayectoria anclada a pose inicial ({x0:.2f},{y0:.2f}) yaw0={yaw0:.2f} rad"
            )
            self.get_logger().info("➡️ Comenzando Lado 1/4")

        # Path para RViz
        self.odom_frame = msg.header.frame_id or self.odom_frame or "odom"
        self.path.header.stamp = self.get_clock().now().to_msg()
        self.path.header.frame_id = self.odom_frame

        self.path.poses.append(current_pose)
        if len(self.path.poses) > self.MAX_POSES:
            self.path.poses = self.path.poses[-self.MAX_POSES:]
        self.path_pub.publish(self.path)

        self.pose = current_pose.pose

    def bumper_callback(self, msg: BumperEvent):
        bumper_name = {0: "Izquierdo", 1: "Central", 2: "Derecho"}.get(msg.bumper, "Desconocido")
        state = "PRESIONADO" if msg.state == 1 else "LIBRE"
        self.get_logger().info(f"🟡 Bumper: {bumper_name} | Estado: {state}")
        if msg.state == 1 and not self.execution_stopped:
            self.execution_stopped = True
            self.stop_robot()
            self.save_path_to_csv("robot_path_stopped.csv")
            self.get_logger().warn("⛔ Bumper activado: robot detenido. Guarda CSV y espera intervención.")
            # Si prefieres backoff y reanudar, aquí puedes añadir lógica de retroceso y poner execution_stopped=False.

    # ===================== Control Loop =====================

    def control_loop(self):
        if self.execution_stopped or self.pose is None or self.current_index >= len(self.waypoints_x):
            return

        now = time.time()
        dt = now - self.last_time
        self.last_time = now

        # Objetivo actual
        tx = self.waypoints_x[self.current_index]
        ty = self.waypoints_y[self.current_index]

        # Pose actual
        x = self.pose.position.x
        y = self.pose.position.y
        yaw = self.get_yaw_from_quaternion(self.pose.orientation)

        # Errores geométricos
        dx, dy = (tx - x), (ty - y)
        dist_err = math.hypot(dx, dy)
        heading_ref = math.atan2(dy, dx)
        yaw_err = math.atan2(math.sin(heading_ref - yaw), math.cos(heading_ref - yaw))

        # Detectar “cerca de esquina”
        NEAR_CORNER_DIST = 0.08
        at_corner = False
        if self.current_side <= 4:
            cut_idx = self.side_breaks[self.current_side - 1]
            at_corner = (abs(self.current_index - cut_idx) <= 2) and (dist_err < NEAR_CORNER_DIST)

        # Entrar a modo esquina: fijar yaw objetivo del siguiente lado
        if at_corner and not self._corner_mode:
            j0 = min(cut_idx + 1, len(self.waypoints_x) - 1)
            j1 = min(j0 + 10, len(self.waypoints_x) - 1)
            vx = self.waypoints_x[j1] - self.waypoints_x[j0]
            vy = self.waypoints_y[j1] - self.waypoints_y[j0]
            self._target_yaw_corner = math.atan2(vy, vx)
            self._corner_mode = True
            self._aligning = True
            self.get_logger().info(f"↩️ Giro de esquina: objetivo yaw={self._target_yaw_corner:.2f} rad")

        # Si estamos en modo esquina, usar error angular fijo hacia yaw objetivo
        if self._corner_mode and self._target_yaw_corner is not None:
            yaw_err = math.atan2(math.sin(self._target_yaw_corner - yaw), math.cos(self._target_yaw_corner - yaw))
            if abs(yaw_err) < 0.10:
                self._corner_mode = False
                self._aligning = False
                self.get_logger().info("✅ Esquina alineada, avanzando siguiente lado")

        # Histéresis de alineación
        if self._aligning:
            if abs(yaw_err) < self.YAW_EXIT_ALIGN:
                self._aligning = False
        else:
            if abs(yaw_err) > self.YAW_ENTER_ALIGN:
                self._aligning = True

        # Control
        if self._aligning:
            lin = 0.0
            ang = self.angular_pid.compute(yaw_err, dt)
            state_txt = "ALINEANDO (giro en sitio)"
        else:
            lin = self.linear_pid.compute(dist_err, dt)
            # mínimo para vencer estática si está lejos
            if dist_err > 0.20 and 0.0 < lin < 0.06:
                lin = 0.06
            ang = self.angular_pid.compute(yaw_err, dt)
            state_txt = "AVANZANDO (recta)"

        # Saturaciones duras
        lin = max(min(lin, self.LIN_MAX), -self.LIN_MAX)
        ang = max(min(ang, self.ANG_MAX), -self.ANG_MAX)

        # Slew-rate angular
        delta = ang - self._last_ang
        max_step = self.ANG_SLEW
        if delta > max_step:
            ang = self._last_ang + max_step
        elif delta < -max_step:
            ang = self._last_ang - max_step
        self._last_ang = ang

        # Publicar
        self.command.linear.x = lin
        self.command.angular.z = ang
        self.publisher_.publish(self.command)

        # Log numérico cada 0.5 s
        if now - self._last_log_t > self._log_dt:
            self._last_log_t = now
            self.get_logger().info(
                f"[Lado {self.current_side}/4] {state_txt} | "
                f"idx={self.current_index} d={dist_err:.3f} yaw_err={yaw_err:.3f} | "
                f"v={lin:.2f} m/s, w={ang:.2f} rad/s (lim v={self.LIN_MAX}, w={self.ANG_MAX})"
            )

        # Cambio de waypoint y de lado
        if dist_err < self.DIST_REACHED:
            self.current_index += 1
            # ¿pasamos al siguiente lado?
            if self.current_side <= 4 and self.current_index == self.side_breaks[self.current_side - 1]:
                if self.current_side < 4:
                    self.current_side += 1
                    self.get_logger().info(f"➡️ Comenzando Lado {self.current_side}/4")
            # ¿terminó el cuadrado?
            if self.current_index >= len(self.waypoints_x):
                self.get_logger().info("✅ Trayectoria completada (cuadrado 1.75 m por lado).")
                self.publisher_.publish(Twist())
                self.save_path_to_csv("robot_path_pid_controller_cuadrado.csv")

    # ===================== Utilidades =====================

    def get_yaw_from_quaternion(self, orientation):
        # yaw desde cuaternión (Z-Y-X)
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y ** 2 + orientation.z ** 2)
        return math.atan2(siny_cosp, cosy_cosp)

    def save_path_to_csvbk(self, filename):
        """Guarda la trayectoria recorrida (x, y, yaw) en CSV."""
        try:
            with open(filename, 'w', newline='') as csvfile:
                writer = csv.DictWriter(csvfile, fieldnames=['x', 'y', 'yaw'])
                writer.writeheader()
                for p in self.path.poses:
                    yaw = self.get_yaw_from_quaternion(p.pose.orientation)
                    writer.writerow({'x': p.pose.position.x, 'y': p.pose.position.y, 'yaw': yaw})
            self.get_logger().info(f"💾 Trayectoria guardada en {filename}")
        except Exception as e:
            self.get_logger().error(f"❌ No se pudo guardar CSV: {e}")

    def save_path_to_csv(self, filename):
        """Guarda la trayectoria recorrida (x, y, yaw) en CSV con PID y timestamp relativo."""
        try:
            # Tiempo de referencia inicial
            t0 = None

            # Parámetros PID
            pid_lin = (self.linear_pid.kp, self.linear_pid.ki, self.linear_pid.kd)
            pid_ang = (self.angular_pid.kp, self.angular_pid.ki, self.angular_pid.kd)

            with open(filename, 'w', newline='') as csvfile:
                fieldnames = ['timestamp', 'x', 'y', 'yaw', 'pid_lineal', 'pid_angular']
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                writer.writeheader()

                for p in self.path.poses:
                    # Calcular yaw
                    yaw = self.get_yaw_from_quaternion(p.pose.orientation)

                    # Obtener timestamp original del mensaje (segundos)
                    t_msg = p.header.stamp.sec + p.header.stamp.nanosec * 1e-9

                    # Inicializar tiempo base
                    if t0 is None:
                        t0 = t_msg

                    # Timestamp relativo (inicio = 0)
                    t_rel = t_msg - t0

                    # Escribir fila
                    writer.writerow({
                        'timestamp': round(t_rel, 3),
                        'x': p.pose.position.x,
                        'y': p.pose.position.y,
                        'yaw': yaw,
                        'pid_lineal': f"{pid_lin[0]:.3f},{pid_lin[1]:.3f},{pid_lin[2]:.3f}",
                        'pid_angular': f"{pid_ang[0]:.3f},{pid_ang[1]:.3f},{pid_ang[2]:.3f}",
                    })

            self.get_logger().info(f"💾 Trayectoria guardada en {filename} (con PID y timestamp relativo)")

        except Exception as e:
            self.get_logger().error(f"❌ No se pudo guardar CSV: {e}")


    def stop_robot(self):
        self.command.linear.x = 0.0
        self.command.angular.z = 0.0
        self.publisher_.publish(self.command)


def main(args=None):
    rclpy.init(args=args)
    node = TurtlebotTrajectoryFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Ctrl+C: saliendo…')
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
