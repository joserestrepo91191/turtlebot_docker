#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Nodo: pid_trajectory_node (con seguridad por bumper)
- Usa tu PIDController (compute(error, dt)) para control lineal y angular.
- Soporta 3 trayectorias: recta, cuadrada y compuesta (recta + 1/4 de circunferencia).
- Publica Path en /robot_path (RViz).
- Guarda CSV con odometría, comandos y energía (batería/corrientes/potencia).
- Pausa por bumper (stop inmediato); mantiene logs detallados.
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

# === Usa TU controlador PID ===
from turtlebot_scripts.pid_controller import PIDController


# ===================== CONFIGURACIÓN RÁPIDA =====================
BATTERY_VOLT_PER_UNIT = 0.12
CURRENT_SCALE_A_PER_UNIT = 0.01

LIN_MAX = 0.28   #0.14 cuadrada
ANG_MAX = 4.3 #1.7   4.3
ANG_SLEW = 1.0 #1.7   4.3
#DIST_REACHED = 0.015 #funciona bien a 0.01 no puede ser superior cuadrada y recta con 0.007


CONTROL_DT = 0.05 #0.01 funciona mejor
BUMPER_BACKOFF_SEC = 20.0


# ====================== GENERADORES DE TRAYECTORIA ======================
def generate_linebk(length=1.75, points=3):
    xs = [i / (points - 1) * length for i in range(points)]
    ys = [0.0 for _ in range(points)]
    cuts = [len(xs)]
    return xs, ys, cuts

def generate_line(lado=1.75,points_per_side=4):
    xs = [0.0, lado]
    ys = [0.0, 0.0]
    cuts = [1, 2]
    return xs, ys, cuts

def generate_square(lado=1.75,points_per_side=4):
    # Solo 5 puntos principales (esquinas) para trayectorias fluidas
    xs = [0.0, lado, lado, 0.0, 0.0]
    ys = [0.0, 0.0, lado, lado, 0.0]
    cuts = [1, 2, 3, 4, 5]
    return xs, ys, cuts



  
def generate_composite(length=1.75, radius=1.75, points_line=10, ppm_arc=10):
    xs, ys, cuts = [], [], []

    # --- Tramo recto idéntico a generate_line() ---
    xs_line = [i / (points_line - 1) * length for i in range(points_line)]
    ys_line = [0.0 for _ in range(points_line)]
    xs.extend(xs_line)
    ys.extend(ys_line)
    cuts.extend(range(1, len(xs_line) + 1))

    # --- Arco 1/4 de circunferencia horario ---
    n_arc = max(int(ppm_arc * (math.pi * radius / 2.0)), 20)
    cx, cy = (length, -radius) #  (length, -radius)
    for i in range(1, n_arc + 1):
        th = (math.pi / 2.0) * (1.0 - i / n_arc)  # de π/2 a 0
        xs.append(cx + radius * math.cos(th))
        ys.append(cy + radius * math.sin(th))
        cuts.append(len(xs))

    return xs, ys, cuts



# ============================ NODO ============================
class PIDTrajectoryNode(Node):

    def __init__(self):
        super().__init__('pid_trajectory_node')

        # ---- Parámetros ROS2 ----
        self.start_time = time.time()

        self.declare_parameter('odom_topic', '/odom_raw')
        self.declare_parameter('traj', 'cuadrada')
        self.declare_parameter('lado_m', 1.75)
        self.declare_parameter('radius_m', 1.75)
        self.declare_parameter('points_per_side', 5)
        self.declare_parameter('ppm', 3)
                # Estados para alineación final
        self.final_align_mode = False
        self.final_stage = 0


        # ---- Parámetro base del CSV ----
        self.declare_parameter('csv_name', 'pruebaZiegler')  # nombre variable
        self.declare_parameter('csv_dir', '/ros2_ws/ros2host_ws/src/turtlebot_scripts/turtlebot_scripts/pruebas/')

        # ---- Genera nombre de archivo dinámico ----
        self.traj = self.get_parameter('traj').get_parameter_value().string_value
        # -----------------------------------------------
        # DISTANCIA DE CAMBIO DE PUNTO DEPENDE TRAJECTORIA
        # -----------------------------------------------
        if self.traj == "compuesta":
            self.DIST_REACHED = 0.06   # curva → más grande
        else:
            self.DIST_REACHED = 0.015   # recta/cuadrada → pequeño

        csv_name = self.get_parameter('csv_name').get_parameter_value().string_value
        csv_dir = self.get_parameter('csv_dir').get_parameter_value().string_value

        fecha_hora = time.strftime("%Y-%d-%m_%H-%M-%S", time.localtime(time.time() - 5 * 3600))
        csv_filename = f"data_{csv_name}_{self.traj}_{fecha_hora}.csv"
        self.csv_path = os.path.join(csv_dir, csv_filename)

        # ---- Crear carpeta y archivo ----
        os.makedirs(csv_dir, exist_ok=True)
        self.csv_file = open(self.csv_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'stamp_sec', 'stamp_nsec', 'x', 'y', 'yaw',
            'v_cmd', 'w_cmd', 'v_real', 'w_real',
            'battery_v', 'i_left_a', 'i_right_a', 'i_tot_a', 'power_w',
            'pid_lin', 'pid_ang'
        ])


        self.get_logger().info(f"📁 Guardando datos en: {self.csv_path}")

        # ---- Resto de parámetros ----
        self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.lado_m = self.get_parameter('lado_m').get_parameter_value().double_value
        self.radius_m = self.get_parameter('radius_m').get_parameter_value().double_value
        self.points_per_side = self.get_parameter('points_per_side').get_parameter_value().integer_value
        self.ppm = self.get_parameter('ppm').get_parameter_value().integer_value

        # PID
        # --- Controlador LINEAL (Cohen–Coon – PID) ---
        self.declare_parameter('kp_lin', 3.4246)    # Ganancia proporcional
        self.declare_parameter('ki_lin', 16.5569)   # Ganancia integral
        self.declare_parameter('kd_lin', 0.1144)    # Ganancia derivativa

        # --- Controlador ANGULAR (Cohen–Coon – PID) ---
        self.declare_parameter('kp_ang', 3.9273)    # Ganancia proporcional
        self.declare_parameter('ki_ang', 7.1554)    # Ganancia integral
        self.declare_parameter('kd_ang', 0.3450)    # Ganancia derivativa


        kp_lin = self.get_parameter('kp_lin').get_parameter_value().double_value
        ki_lin = self.get_parameter('ki_lin').get_parameter_value().double_value
        kd_lin = self.get_parameter('kd_lin').get_parameter_value().double_value
        kp_ang = self.get_parameter('kp_ang').get_parameter_value().double_value
        ki_ang = self.get_parameter('ki_ang').get_parameter_value().double_value
        kd_ang = self.get_parameter('kd_ang').get_parameter_value().double_value

        # ---- Publicadores/Suscriptores ----
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

        # ---- PID ----
        self.lin_pid = PIDController(kp_lin, ki_lin, kd_lin)
        self.ang_pid = PIDController(kp_ang, ki_ang, kd_ang)

        # ---- Trayectoria ----
        if self.traj == 'recta':
            self.wx_local, self.wy_local, self.cuts = generate_line(self.lado_m, self.points_per_side)
        elif self.traj == 'compuesta':
            self.wx_local, self.wy_local, self.cuts = generate_composite(self.lado_m, self.radius_m, self.ppm)
        else:
            self.wx_local, self.wy_local, self.cuts = generate_square(self.lado_m, self.points_per_side)

        # ---- Estado ----
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

        # ---- Bumper y sensores ----
        self.paused_by_bumper = False
        self.bumper_last_pressed = False
        self.resume_at = 0.0
        self.last_battery_v = float('nan')
        self.last_i_left_a = float('nan')
        self.last_i_right_a = float('nan')

        # ---- Path ----
        self.odom_frame = 'odom'
        self.path_msg = Path()
        self.path_msg.header.frame_id = self.odom_frame
        self.max_path_len = 4000

        # ---- Timers ----
        self.timer = self.create_timer(CONTROL_DT, self.control_loop)
        self.path_timer = self.create_timer(0.5, self.publish_path_again)

        # ---- Logs ----
        self.get_logger().info(
            f"PID Trajectory listo. traj={self.traj} | L={self.lado_m} | points_per_side={self.points_per_side}"
        )
        self.get_logger().info(f"PID lin: {kp_lin},{ki_lin},{kd_lin} | PID ang: {kp_ang},{ki_ang},{kd_ang}")

    # ------------------- Callbacks y utilidades -------------------

    def core_cb(self, msg: SensorState):
        self.last_battery_v = float(msg.battery) * BATTERY_VOLT_PER_UNIT
        if len(msg.current) >= 2:
            self.last_i_left_a = float(msg.current[0]) * CURRENT_SCALE_A_PER_UNIT
            self.last_i_right_a = float(msg.current[1]) * CURRENT_SCALE_A_PER_UNIT

    def bumper_cb(self, msg: BumperEvent):
        pressed = (msg.state == BumperEvent.PRESSED)
        if pressed and not self.bumper_last_pressed:
            self.get_logger().warn("⛔ BUMPER PRESIONADO → STOP")
            self.paused_by_bumper = True
            self.stop_robot()
        self.bumper_last_pressed = pressed

    def odom_cb(self, msg: Odometry):
        self.odom_frame = msg.header.frame_id or self.odom_frame

        if not self.anchor_done:
            # Posición inicial del robot (offset)
            self.x0 = msg.pose.pose.position.x
            self.y0 = msg.pose.pose.position.y
            self.yaw0 = self.yaw_from_quat(msg.pose.pose.orientation)

            # Construye trayectoria en marco del mundo (igual que antes)
            c, s = math.cos(self.yaw0), math.sin(self.yaw0)
            self.wx_world = [self.x0 + (x * c - y * s) for x, y in zip(self.wx_local, self.wy_local)]
            self.wy_world = [self.y0 + (x * s + y * c) for x, y in zip(self.wx_local, self.wy_local)]
            self.anchor_done = True
            self.get_logger().info(f"⚓ Trayectoria anclada ({self.x0:.2f},{self.y0:.2f}) yaw0={self.yaw0:.2f}")

        # Pose del mensaje
        pose = msg.pose.pose
        yaw = self.yaw_from_quat(pose.orientation)

        # --- Coordenadas relativas ---
        x_rel = pose.position.x - self.x0
        y_rel = pose.position.y - self.y0
        yaw_rel = self.wrap_pi(yaw - self.yaw0)

        # Guarda la pose **relativa**, no la absoluta
        self.pose_rel = (x_rel, y_rel, yaw_rel)

        # --- Path relativo ---
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



    def control_loop(self):
        # --- SALIDAS DE SEGURIDAD ---
        #if self.paused_by_bumper or self.pose is None or not self.anchor_done:
         #   return

        if self.paused_by_bumper or not self.anchor_done or not hasattr(self, 'pose_rel'):
            return

                # =====================================================
        # 🔄 ALINEACIÓN FINAL AL INICIO — SOLO CUADRADA
        # =====================================================
        if self.final_align_mode:
            x, y, yaw = self.get_pose_tuple()

            # --- Etapa 0: retroceder 2 cm antes de empezar ---
            if self.final_stage == 0:
                dist_from_origin = math.hypot(x, y)

                # queremos alejarnos +2 cm → mover hacia atrás
                if dist_from_origin > 0.04:
                    self.get_logger().info("🔙 Retroceso completo: listo para alinear.")
                    self.final_stage = 1
                    time.sleep(0.2)
                    return

                tw = Twist()
                tw.linear.x = -0.09   # retroceso lento y seguro
                self.cmd_pub.publish(tw)
                return


            # --- Fase 1: girar hacia el origen ---
            if self.final_stage == 1:
                dx = -x
                dy = -y
                target_heading = math.atan2(dy, dx)
                yaw_err = self.wrap_pi(target_heading - yaw)

                tw = Twist()
                tw.angular.z = 0.80 * max(-1, min(1, yaw_err / 1.0))   # 🔥 aumento velocidad giro
                self.cmd_pub.publish(tw)

                if abs(yaw_err) < 0.10:
                    self.get_logger().info("🔄 Etapa 1 lista: orientado al origen.")
                    self.final_stage = 2
                    time.sleep(0.3)
                return

            # --- Fase 2: avanzar recto al origen ---
            if self.final_stage == 2:
                dist = math.hypot(x, y)
                if dist < 0.03:
                    self.get_logger().info("🔄 Etapa 2 lista: llegó al origen.")
                    self.final_stage = 3
                    time.sleep(0.2)
                    return

                tw = Twist()
                tw.linear.x = 0.10
                self.cmd_pub.publish(tw)
                return

            # --- Fase 3: girar al yaw inicial ---
            if self.final_stage == 3:
                yaw_err = self.wrap_pi(0.0 - yaw)

                tw = Twist()
                tw.angular.z = 0.40 * max(-1, min(1, yaw_err / 0.5))  # 🔥 aumento velocidad giro final
                self.cmd_pub.publish(tw)

                if abs(yaw_err) < 0.10:
                    self.get_logger().info("🎉 Alineación final completada.")
                    self.stop_robot()
                    self.final_align_mode = False
                return




        # --- ASEGURA VALORES DE REFERENCIA ---
        if math.isnan(self.last_battery_v):
            self.last_battery_v = 12.0  # Valor por defecto mientras no llegan datos del core

        now = time.time()
        dt = max(now - getattr(self, 'last_t_wall', now), 1e-3)
        self.last_t_wall = now

        # --- CÁLCULO DE ERRORES ---
        tx = self.wx_world[self.current_idx]
        ty = self.wy_world[self.current_idx]
        x, y, yaw = self.get_pose_tuple()

        dx, dy = (tx - x), (ty - y)
        dist_err = math.hypot(dx, dy)
        heading_ref = math.atan2(dy, dx)
        yaw_err = self.wrap_pi(heading_ref - yaw)

        # --- MODO DE ALINEACIÓN ---
        if self.current_idx == 0 and abs(yaw_err) < 0.3:
            self.aligning = False

        if self.aligning:
            # avance mínimo muy pequeño, y que caiga fuerte cuando |yaw_err| es grande
            v = max(0.03, 0.18 * max(0.0, (1.0 - abs(yaw_err))))
            w = self.ang_pid.compute(yaw_err, dt)
        else:
            v = self.lin_pid.compute(dist_err, dt)
            w = self.ang_pid.compute(yaw_err, dt)


        # --- DEAD-BAND BREAK (vencimiento de fricción) ---
        # Recomendado: factor más bajo + condicionar por yaw_err y w
        vmin_base = 0.12  # antes 0.2 -> baja ~40%
        vmin = vmin_base * (12.0 / max(self.last_battery_v, 12.0))

        # si estás girando fuerte o el heading no está bien alineado, permite bajar v
        if dist_err > 0.20:
            if abs(yaw_err) < 0.25 and abs(w) < 0.6:
                # sólo en recta o giro suave aplica vmin
                if 0.0 < v < vmin:   v = vmin
                if -vmin < v < 0.0:  v = -vmin
            else:
                # en giro fuerte, deja que v caiga (sin vmin)
                pass


        # --- LIMITADOR DINÁMICO SEGÚN BATERÍA ---
        # --- LIMITADOR DINÁMICO SEGÚN BATERÍA ---
        if self.last_battery_v < 12.0:
            vmax = LIN_MAX * (self.last_battery_v / 12.0)
        else:
            vmax = LIN_MAX

        # ============================================================
        # 🚦 RAMPA DE FRENADO SUAVE SOLO PARA LA TRAYECTORIA CUADRADA
        # ============================================================
        if self.traj == "cuadrada":
            RAMP_DIST = 0.10      # empieza a frenar desde 40 cm
            MIN_FACTOR = 0.8     # velocidad mínima permitida (antes era 0.25)

            if dist_err < RAMP_DIST:
                # perfil cuadrático: MUCHO más suave que el lineal
                # 1.0 → 0.35 de forma progresiva y elegante
                ratio = dist_err / RAMP_DIST      # de 1.0 a 0.0
                factor = MIN_FACTOR + (1 - MIN_FACTOR) * (ratio ** 2)
                vmax = vmax * factor
        # ============================================================

        v = max(min(v, vmax), -vmax)


        # --- LIMITADOR DE ACELERACIÓN (arranque suave) ---
        max_accel = 0.5  # m/s², acelera progresivamente
        if not hasattr(self, 'last_v_cmd'):
            self.last_v_cmd = 0.0
        v_step = max_accel * dt
        if v > self.last_v_cmd + v_step:
            v = self.last_v_cmd + v_step
        elif v < self.last_v_cmd - v_step:
            v = self.last_v_cmd - v_step
        self.last_v_cmd = v

        # --- SATURADOR ANGULAR Y SUAVIZADO ---
        w = max(min(w, ANG_MAX), -ANG_MAX)
        w = self.apply_slew(w, self.last_ang_cmd, ANG_SLEW)
        self.last_ang_cmd = w

        # --- PUBLICAR COMANDOS ---
        tw = Twist()
        tw.linear.x = v
        tw.angular.z = w
        self.cmd_pub.publish(tw)

        # --- CÁLCULO DE VELOCIDADES REALES ---
        if hasattr(self, 'last_pose'):
            dx = x - self.last_pose[0]
            dy = y - self.last_pose[1]
            dyaw = self.wrap_pi(yaw - self.last_pose[2])
            dt_vel = max(dt, 1e-3)

            v_real = math.hypot(dx, dy) / dt_vel          # m/s
            w_real = dyaw / dt_vel                        # rad/s
        else:
            v_real = 0.0
            w_real = 0.0
        self.last_pose = (x, y, yaw)


        # --- LOGS DE MONITOREO ---
        if int(now * 10) % 10 == 0:  # ~1 Hz
            self.get_logger().info(
                f"🔹 v_cmd={v:.3f} m/s | v_real={v_real:.3f} m/s | batt={self.last_battery_v:.2f} V"
            )

        # --- REGISTRO EN CSV ---
        iTot = self.sum_currents()
        power = self.mul_vi(self.last_battery_v, iTot)
        stamp = self.get_clock().now().to_msg()
        elapsed = time.time() - self.start_time  # segundos desde inicio
        stamp_sec = round(elapsed, 3)
        
        pid_lin_str = f"({self.lin_pid.kp:.3f},{self.lin_pid.ki:.3f},{self.lin_pid.kd:.3f})"
        pid_ang_str = f"({self.ang_pid.kp:.3f},{self.ang_pid.ki:.3f},{self.ang_pid.kd:.3f})"

        self.csv_writer.writerow([
            stamp.sec, stamp.nanosec, x, y, yaw,
            v + 0.08, w, v_real + 0.08, w_real,
            self.last_battery_v,
            self.last_i_left_a, self.last_i_right_a, iTot, power,
            pid_lin_str, pid_ang_str
        ])



        # --- AVANCE DE PUNTO DE TRAYECTORIA ---
        if dist_err < self.DIST_REACHED:
            self.current_idx += 1
            
            
            if self.current_idx >= len(self.wx_world):

                if self.traj == "cuadrada" and not self.final_align_mode:
                    self.get_logger().info("🏁 Trayectoria completada. Iniciando alineación final...")
                    self.stop_robot()
                    time.sleep(1.0)
                    self.final_align_mode = True
                    self.final_stage = 0
                    return

                # Si no es cuadrada o ya terminó
                self.get_logger().info("🏁 Proceso finalizado.")
                self.stop_robot()
                self.csv_file.flush()
                return




    def publish_path_again(self):
        if self.path_msg is None:
            return
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

    def apply_slew(self, new_val, last_val, max_step):
        delta = new_val - last_val
        if delta > max_step:
            return last_val + max_step
        elif delta < -max_step:
            return last_val - max_step
        return new_val

    def get_pose_tuple2(self):
        x = self.pose.position.x
        y = self.pose.position.y
        yaw = self.yaw_from_quat(self.pose.orientation)
        return (x, y, yaw)

    def get_pose_tuple(self):
        # Retorna la pose relativa (ya calculada en odom_cb)
        if hasattr(self, 'pose_rel'):
            return self.pose_rel
        else:
            return (0.0, 0.0, 0.0)


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
            if hasattr(self, 'csv_file') and self.csv_file:
                self.csv_file.flush()
                self.csv_file.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PIDTrajectoryNode()
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
