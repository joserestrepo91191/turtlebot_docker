#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Nodo: pid_trajectory_node (parametrización continua)
- Control PID lineal y angular sobre error a una trayectoria continua.
- Soporta: recta, cuadrada y compuesta (recta + 1/4 circunferencia).
- Publica Path en /robot_path.
- Guarda CSV con odometría, comandos y energía.
- Se detiene si se activa bumper.

Parámetros:
  traj: 'recta' | 'cuadrada' | 'compuesta'
  lado_m: 1.75
  radius_m: 1.75
  v_ref: velocidad lineal de referencia (m/s)
  kp_lin, ki_lin, kd_lin
  kp_ang, ki_ang, kd_ang
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
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

# Importa tu controlador PID
from turtlebot_scripts.pid_controller import PIDController

# ----------------- Escalas de sensores -----------------
BATTERY_VOLT_PER_UNIT = 0.1
CURRENT_SCALE_A_PER_UNIT = 0.01

# ----------------- Límites -----------------
LIN_MAX = 0.44
ANG_MAX = 2.0
CONTROL_DT = 0.033   # 30 Hz aprox.

# Seguridad bumper
BUMPER_BACKOFF_SEC = 20.0  # por si deseas reanudar tras liberar (aquí solo pausa)

# ========================================================
class PIDTrajectoryNode(Node):
    def __init__(self):
        super().__init__('pid_trajectory_node')

        # ---- Parámetros ROS2 ----
        self.declare_parameter('odom_topic', '/odom_raw')
        #self.declare_parameter('csv_path', '/ros2_ws/ros2host_ws/src/turtlebot_scripts/turtlebot_scripts/data.csv')

        self.declare_parameter('traj', 'cuadrada')
        self.declare_parameter('lado_m', 1.75)
        self.declare_parameter('radius_m', 1.75)
        self.declare_parameter('v_ref', 0.15)  # velocidad de referencia

        self.declare_parameter('kp_lin', 2.4890) #1.2
        self.declare_parameter('ki_lin', 1.1573) #0.0
        self.declare_parameter('kd_lin', 0.3862) #0.05

        self.declare_parameter('kp_ang', 3.2052) #3.0
        self.declare_parameter('ki_ang', 1.4903) #0.0
        self.declare_parameter('kd_ang', 0.4974) #1.0

        """
        MEJORES GANANCIAS ENCONTRADAS:
        kp_lin      0.800000
        ki_lin      0.000000
        kd_lin      0.020000
        kp_ang      2.000000
        ki_ang      0.000000
        kd_ang      0.500000
        J_tot     132.264075
        Name: 0, dtype: float64

        
            --- GANANCIAS RECOMENDADAS ---

        Modo: lineal
        Método: ZN
        Kp = 4.7789
        Ki = 9.7768
        Kd = 0.5840

        Modo: angular
        Método: ZN
        Kp = 6.1541
        Ki = 12.5898
        Kd = 0.7520

        ---------------

        Modo: lineal
        Método: Tyreus-Luyben
        Kp = 2.4890
        Ki = 1.1573
        Kd = 0.3862

        Modo: angular
        Método: Tyreus-Luyben
        Kp = 3.2052
        Ki = 1.4903
        Kd = 0.4974



        """

        # ---- Lee parámetros ----
        self.odom_topic = self.get_parameter('odom_topic').value
        self.traj       = self.get_parameter('traj').value
        self.lado_m     = self.get_parameter('lado_m').value
        self.radius_m   = self.get_parameter('radius_m').value
        self.v_ref      = self.get_parameter('v_ref').value

        kp_lin = self.get_parameter('kp_lin').value
        ki_lin = self.get_parameter('ki_lin').value
        kd_lin = self.get_parameter('kd_lin').value

        kp_ang = self.get_parameter('kp_ang').value
        ki_ang = self.get_parameter('ki_ang').value
        kd_ang = self.get_parameter('kd_ang').value

        # ---- Publicadores/Suscriptores ----
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        path_qos = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=1,
                              reliability=QoSReliabilityPolicy.RELIABLE,
                              durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.path_pub = self.create_publisher(Path, '/robot_path', path_qos)

        self.create_subscription(Odometry, self.odom_topic, self.odom_cb, 10)
        self.create_subscription(SensorState, '/sensors/core', self.core_cb, 10)
        self.create_subscription(BumperEvent, '/events/bumper', self.bumper_cb, 10)

        # ---- PID ----
        self.lin_pid = PIDController(kp_lin, ki_lin, kd_lin)
        self.ang_pid = PIDController(kp_ang, ki_ang, kd_ang)

        # ---- Estado ----
        self.pose = None
        self.anchor_done = False
        self.x0, self.y0, self.yaw0 = 0.0, 0.0, 0.0

        self.s_ref = 0.0  # parámetro de trayectoria
        self.total_length = self.compute_total_length()

        # ---- Energía ----
        self.last_battery_v = float('nan')
        self.last_i_left_a  = float('nan')
        self.last_i_right_a = float('nan')

        # ---- Path ----
        self.path_msg = Path()
        self.path_msg.header.frame_id = 'odom'
        self.max_path_len = 4000

        # ---- CSV ----
        #os.makedirs(os.path.dirname(self.csv_path), exist_ok=True)
        #self.csv_file = open(self.csv_path, 'w', newline='')
        #self.csv_writer = csv.writer(self.csv_file)
        #self.csv_writer.writerow([
        #    'stamp_sec','stamp_nsec','x','y','yaw',
        #    'v_cmd','w_cmd','battery_v','i_left_a','i_right_a','i_tot_a','power_w'
        #])


        # ---- Parámetro base del CSV ----
        self.declare_parameter('csv_name', 'pruebaZiegler')  # nombre variable
        self.declare_parameter('csv_dir', '/ros2_ws/ros2host_ws/src/turtlebot_scripts/turtlebot_scripts/')

        # ---- Genera nombre de archivo dinámico ----
        csv_name = self.get_parameter('csv_name').value
        csv_dir  = self.get_parameter('csv_dir').value
        fecha_hora = time.strftime("%Y-%d-%m_%H-%M-%S", time.localtime(time.time() - 5*3600))

        csv_filename = f"data_{csv_name}_{self.traj}_{fecha_hora}.csv"
        self.csv_path = os.path.join(csv_dir, csv_filename)

        # ---- Crear carpeta y archivo ----
        os.makedirs(csv_dir, exist_ok=True)
        self.csv_file = open(self.csv_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'stamp_sec','stamp_nsec','x','y','yaw',
            'v_cmd','w_cmd','battery_v','i_left_a','i_right_a','i_tot_a','power_w'
        ])
        self.get_logger().info(f"📁 Guardando datos en: {self.csv_path}")


        # ---- Timer ----
        self.last_t_wall = time.time()
        self.timer = self.create_timer(CONTROL_DT, self.control_loop)

        self.get_logger().info(
            f"Trayectoria continua: {self.traj}, L={self.lado_m}, R={self.radius_m}, v_ref={self.v_ref}"
        )
        self.get_logger().info(
            f"PID lin: {kp_lin},{ki_lin},{kd_lin} | PID ang: {kp_ang},{ki_ang},{kd_ang}"
        )

    # ------------------- Sensores -------------------
    def core_cb(self, msg: SensorState):
        self.last_battery_v = float(msg.battery) * BATTERY_VOLT_PER_UNIT
        if len(msg.current) >= 2:
            self.last_i_left_a  = float(msg.current[0]) * CURRENT_SCALE_A_PER_UNIT
            self.last_i_right_a = float(msg.current[1]) * CURRENT_SCALE_A_PER_UNIT

    def bumper_cb(self, msg: BumperEvent):
        pressed = (msg.state == BumperEvent.PRESSED)
        name = {0: "Izquierdo", 1: "Central", 2: "Derecho"}.get(msg.bumper, "Desconocido")
        if pressed and not self.bumper_last_pressed:
            self.get_logger().warn(f"⛔ BUMPER PRESIONADO ({name}) → PAUSA inmediata")
            self.paused_by_bumper = True
            self.stop_robot()
            # marca CSV (fila con v=w=0)
            stamp = self.get_clock().now().to_msg()
            x,y,yaw = self.get_pose_tuple()
            iTot = self.sum_currents()
            power = self.mul_vi(self.last_battery_v, iTot)
            self.csv_writer.writerow([stamp.sec, stamp.nanosec, x, y, yaw, 0.0, 0.0,
                                      self.last_battery_v, self.last_i_left_a, self.last_i_right_a,
                                      iTot, power])
        elif (not pressed) and self.bumper_last_pressed:
            # Si quisieras reanudar tras backoff, habilita esta lógica
            self.resume_at = time.time() + BUMPER_BACKOFF_SEC
            self.get_logger().info(f"🟢 BUMPER LIBRE ({name}) → (backoff {BUMPER_BACKOFF_SEC:.1f} s)")
        self.bumper_last_pressed = pressed
    def odom_cb(self, msg: Odometry):
        self.pose = msg.pose.pose
        if not self.anchor_done:
            self.x0 = self.pose.position.x
            self.y0 = self.pose.position.y
            self.yaw0 = self.yaw_from_quat(self.pose.orientation)
            self.anchor_done = True
            self.get_logger().info(
                f"⚓ Trayectoria anclada en ({self.x0:.2f},{self.y0:.2f}) yaw0={self.yaw0:.2f}"
            )

        ps = PoseStamped()
        ps.header = msg.header
        ps.pose = self.pose
        self.path_msg.poses.append(ps)
        if len(self.path_msg.poses) > self.max_path_len:
            self.path_msg.poses = self.path_msg.poses[-self.max_path_len:]
        self.path_pub.publish(self.path_msg)

    # ------------------- Control -------------------
    def control_loop(self):
        if self.pose is None or not self.anchor_done:
            return

        now = time.time()
        dt = max(now - self.last_t_wall, 1e-3)
        self.last_t_wall = now

        # avanzar parámetro
        self.s_ref += self.v_ref * dt
        if self.s_ref > self.total_length:
            self.get_logger().info("🏁 Trayectoria completada.")
            self.stop_robot()
            return

        # referencia deseada
        xd, yd, yawd = self.get_ref(self.s_ref)

        # pose actual
        x, y, yaw = self.get_pose_tuple()
        dx, dy = xd - x, yd - y
        dist_err = math.hypot(dx, dy)
        yaw_err = self.wrap_pi(yawd - yaw)

        # PID
        v = self.lin_pid.compute(dist_err, dt)
        w = self.ang_pid.compute(yaw_err, dt)

        v = max(min(v, LIN_MAX), -LIN_MAX)
        w = max(min(w, ANG_MAX), -ANG_MAX)

        tw = Twist()
        tw.linear.x = v
        tw.angular.z = w
        self.cmd_pub.publish(tw)

        # CSV
        iTot = self.sum_currents()
        power = self.mul_vi(self.last_battery_v, iTot)
        stamp = self.get_clock().now().to_msg()
        self.csv_writer.writerow([stamp.sec, stamp.nanosec, x, y, yaw, v, w,
                                  self.last_battery_v, self.last_i_left_a, self.last_i_right_a,
                                  iTot, power])

    # ------------------- Referencias -------------------
    def get_ref(self, s):
        if self.traj == 'recta':
            return self.ref_line(s)
        elif self.traj == 'cuadrada':
            return self.ref_square(s)
        elif self.traj == 'compuesta':
            return self.ref_composite(s)
        else:
            return self.ref_line(s)

    def ref_line(self, s):
        s = min(s, self.lado_m)
        xd = self.x0 + s*math.cos(self.yaw0)
        yd = self.y0 + s*math.sin(self.yaw0)
        yawd = self.yaw0
        return xd, yd, yawd

    def ref_square(self, s):
        L = self.lado_m
        perim = 4*L
        s = min(s, perim)
        seg = int(s // L)
        offset = s % L
        if seg == 0:   # lado 1
            xd = self.x0 + offset*math.cos(self.yaw0)
            yd = self.y0 + offset*math.sin(self.yaw0)
            yawd = self.yaw0
        elif seg == 1: # lado 2
            xd = self.x0 + L*math.cos(self.yaw0) - offset*math.sin(self.yaw0)
            yd = self.y0 + L*math.sin(self.yaw0) + offset*math.cos(self.yaw0)
            yawd = self.yaw0 + math.pi/2
        elif seg == 2: # lado 3
            xd = self.x0 + (L-offset)*math.cos(self.yaw0) - L*math.sin(self.yaw0)
            yd = self.y0 + (L-offset)*math.sin(self.yaw0) + L*math.cos(self.yaw0)
            yawd = self.yaw0 + math.pi
        else:          # lado 4
            xd = self.x0 - offset*math.sin(self.yaw0)
            yd = self.y0 + (L-offset)*math.cos(self.yaw0)
            yawd = self.yaw0 - math.pi/2
        return xd, yd, self.wrap_pi(yawd)

    def ref_composite(self, s):
        L = self.lado_m
        R = self.radius_m
        arc_len = math.pi*R/2
        total = L + arc_len
        s = min(s, total)
        if s <= L:
            # tramo recto
            xd = self.x0 + s*math.cos(self.yaw0)
            yd = self.y0 + s*math.sin(self.yaw0)
            yawd = self.yaw0
        else:
            # arco CCW
            sa = s - L
            theta = sa / R  # ángulo recorrido
            cx = self.x0 + L*math.cos(self.yaw0) - R*math.sin(self.yaw0)
            cy = self.y0 + L*math.sin(self.yaw0) + R*math.cos(self.yaw0)
            xd = cx + R*math.cos(self.yaw0 - math.pi/2 + theta)
            yd = cy + R*math.sin(self.yaw0 - math.pi/2 + theta)
            yawd = self.yaw0 + theta
        return xd, yd, self.wrap_pi(yawd)

    def compute_total_length(self):
        if self.traj == 'recta':
            return self.lado_m
        elif self.traj == 'cuadrada':
            return 4*self.lado_m
        elif self.traj == 'compuesta':
            return self.lado_m + math.pi*self.radius_m/2
        else:
            return self.lado_m

    # ------------------- Utilidades -------------------
    def yaw_from_quat(self, q):
        siny_cosp = 2 * (q.w*q.z + q.x*q.y)
        cosy_cosp = 1 - 2 * (q.y*q.y + q.z*q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def wrap_pi(self, a):
        return math.atan2(math.sin(a), math.cos(a))

    def get_pose_tuple(self):
        if self.pose is None:
            return (float('nan'), float('nan'), float('nan'))
        x = self.pose.position.x
        y = self.pose.position.y
        yaw = self.yaw_from_quat(self.pose.orientation)
        return (x, y, yaw)

    def sum_currents(self):
        if math.isnan(self.last_i_left_a) or math.isnan(self.last_i_right_a):
            return float('nan')
        return self.last_i_left_a + self.last_i_right_a

    def mul_vi(self, v, i):
        if math.isnan(v) or math.isnan(i):
            return float('nan')
        return v * i

    def stop_robot(self):
        tw = Twist()
        self.cmd_pub.publish(tw)

    def destroy_node(self):
        try:
            if self.csv_file:
                self.csv_file.flush()
                self.csv_file.close()
        except Exception:
            pass
        super().destroy_node()

# ============================ MAIN ============================
def main(args=None):
    rclpy.init(args=args)
    node = PIDTrajectoryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Ctrl+C → Deteniendo robot…")
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
