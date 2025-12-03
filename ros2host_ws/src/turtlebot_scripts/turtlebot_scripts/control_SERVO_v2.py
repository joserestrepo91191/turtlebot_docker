#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Nodo: servo_trajectory_node
- Igual estructura del PID pero usando control SERVO INTEGRADOR.
- La lógica de trayectoria, CSV, path, bumper y alineación final se mantiene.
- ÚNICAMENTE cambia el cómputo de v_cmd y w_cmd.
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

# ===========================================
# IMPORTAMOS TU CONTROLADOR SERVO
# ===========================================
from turtlebot_scripts.servo_controller import ServoIntegratorController


# ===========================================
# CONSTANTES
# ===========================================
BATTERY_VOLT_PER_UNIT = 0.12
CURRENT_SCALE_A_PER_UNIT = 0.05

LIN_MAX = 0.27
ANG_MAX = 2.3
ANG_SLEW = 2.5

CONTROL_DT = 0.01


# ===========================================
# GENERADORES DE TRAYECTORIA
# ===========================================
def generate_line(lado=1.75,points_per_side=4):
    xs = [0.0, lado]
    ys = [0.0, 0.0]
    cuts = [1, 2]
    return xs, ys, cuts


def generate_square(lado=1.75, points_per_side=4):
    xs = [0.0,lado,lado,0.0,0.0]
    ys = [0.0,0.0,lado,lado,0.0]
    cuts = [1,2,3,4,5]
    return xs, ys, cuts


def generate_square1(lado=1.75, points_per_side=10):
    xs = []
    ys = []

    N = points_per_side

    for i in range(N):
        xs.append(i * (lado / (N - 1)))
        ys.append(0.0)

    for i in range(1, N):
        xs.append(lado)
        ys.append(i * (lado / (N - 1)))

    for i in range(1, N):
        xs.append(lado - i * (lado / (N - 1)))
        ys.append(lado)

    for i in range(1, N):
        xs.append(0.0)
        ys.append(lado - i * (lado / (N - 1)))

    cuts = list(range(1, len(xs) + 1))
    return xs, ys, cuts


def generate_composite(length=1.75, radius=1.75, pts=10, ppm_arc=10):
    xs, ys, cuts = [], [], []
    
    xs_line = [i/(pts-1)*length for i in range(pts)]
    ys_line = [0]*pts
    xs.extend(xs_line)
    ys.extend(ys_line)
    cuts.extend(range(1,pts+1))

    n_arc = max(int(ppm_arc*(math.pi*radius/2)),20)
    cx, cy = (length,-radius)

    for i in range(1,n_arc+1):
        th = (math.pi/2)*(1-i/n_arc)
        xs.append(cx + radius*math.cos(th))
        ys.append(cy + radius*math.sin(th))
        cuts.append(len(xs))

    return xs, ys, cuts


# ===========================================
# COEFICIENTES DEL SERVO (TU SINTONIZACIÓN)
# ===========================================
A_L = [1.48, -0.5014, 0.02145]
B_L = [2.686, -3.147, 1.047]
C_L = [2.686, -3.377, 1.277]

A_A = [0.0, 0.0, 0.0]
B_A = [5.0, -1.0, 0.0]
C_A = [6.627, 0.0, 0.9911]



# =====================================================
# ===================== NODO PRINCIPAL ===============
# =====================================================
class ServoTrajectoryNode(Node):

    def __init__(self):
        super().__init__("servo_trajectory_node")

        self.start_time = time.time()

        # ---------------------------------------
        # PARÁMETROS ROS2
        # ---------------------------------------
        self.declare_parameter("traj","cuadrada")
        self.declare_parameter("lado_m",1.75)
        self.declare_parameter("points_per_side",4)
        self.declare_parameter("radius_m",1.75)
        self.declare_parameter("ppm",3)

        self.declare_parameter("csv_name","pruebaServo")
        self.declare_parameter(
            "csv_dir",
            "/ros2_ws/ros2host_ws/src/turtlebot_scripts/turtlebot_scripts/pruebas/"
        )

        self.traj = self.get_parameter("traj").value
        self.lado_m = self.get_parameter("lado_m").value
        self.points_per_side = self.get_parameter("points_per_side").value
        self.radius_m = self.get_parameter("radius_m").value
        self.ppm = self.get_parameter("ppm").value

        # ---------------------------------------
        # DISTANCIA DE CAMBIO DE PUNTO
        # ---------------------------------------
        if self.traj == "compuesta":
            self.DIST_REACHED = 0.06
        else:
            self.DIST_REACHED = 0.022  #22

        # ---------------------------------------
        # NOMBRE CSV
        # ---------------------------------------
        csv_dir = self.get_parameter("csv_dir").value
        csv_base = self.get_parameter("csv_name").value

        fecha = time.strftime("%Y-%d-%m_%H-%M-%S",time.localtime(time.time()-5*3600))
        csv_name = f"data_{csv_base}_{self.traj}_{fecha}.csv"
        self.csv_path = os.path.join(csv_dir,csv_name)
        
        os.makedirs(csv_dir,exist_ok=True)
        self.csv_file = open(self.csv_path,"w",newline="")
        self.csv_writer = csv.writer(self.csv_file)

        self.csv_writer.writerow([
            "stamp_sec","stamp_nsec",
            "x","y","yaw",
            "v_cmd","w_cmd","v_real","w_real",
            "battery_v","i_left","i_right","i_tot","power",
            "servo_lin","servo_ang"
        ])

        self.get_logger().info(f"📁 Guardando datos en: {self.csv_path}")

        # ---------------------------------------
        # CONTROLADORES SERVO
        # ---------------------------------------
        self.servo_lin = ServoIntegratorController(A_L,B_L,C_L)
        self.servo_ang = ServoIntegratorController(A_A,B_A,C_A)

        # ---------------------------------------
        # TRAYECTORIA
        # ---------------------------------------
        if self.traj == "recta":
            self.wx_local, self.wy_local, self.cuts = generate_line(self.lado_m,self.points_per_side)
        elif self.traj == "compuesta":
            self.wx_local, self.wy_local, self.cuts = generate_composite(self.lado_m,self.radius_m,self.ppm)
        else:
            self.wx_local, self.wy_local, self.cuts = generate_square(self.lado_m,self.points_per_side)

        # ---------------------------------------
        # ESTADO INTERNO
        # ---------------------------------------
        self.anchor_done = False
        self.pose_rel = None
        self.last_t = time.time()
        self.current_idx = 0
        self.last_pose = None
        self.last_ang_cmd = 0.0

        # ---------------------------------------
        # SUSCRIPTORES Y PUBLICADORES
        # ---------------------------------------
        self.cmd_pub = self.create_publisher(Twist,"/cmd_vel",10)

        path_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        self.path_pub = self.create_publisher(Path,"/robot_path",path_qos)

        self.create_subscription(Odometry,"/odom_raw",self.odom_cb,10)
        self.create_subscription(SensorState,"/sensors/core",self.core_cb,10)
        self.create_subscription(BumperEvent,"/events/bumper",self.bumper_cb,10)

        self.path_msg = Path()
        self.path_msg.header.frame_id = "odom"

        # TIMERS
        self.timer = self.create_timer(CONTROL_DT, self.control_loop)
        self.path_timer = self.create_timer(0.5, self.publish_path)

        self.get_logger().info("🔧 Nodo Servo listo.")

    # ===================================================
    # CALLBACKS
    # ===================================================
    def core_cb(self,msg):
        self.last_batt = msg.battery * BATTERY_VOLT_PER_UNIT
        self.i_left = msg.current[0] * CURRENT_SCALE_A_PER_UNIT
        self.i_right = msg.current[1] * CURRENT_SCALE_A_PER_UNIT

    def bumper_cb(self,msg):
        if msg.state == BumperEvent.PRESSED:
            self.get_logger().warn("⛔ BUMPER PRESIONADO → STOP")
            self.stop_robot()

    def odom_cb(self,msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = self.quat_to_yaw(msg.pose.pose.orientation)

        if not self.anchor_done:
            self.x0 = x
            self.y0 = y
            self.yaw0 = yaw

            c,s = math.cos(yaw), math.sin(yaw)
            self.wx_world = [self.x0+(px*c - py*s) for px,py in zip(self.wx_local,self.wy_local)]
            self.wy_world = [self.y0+(px*s + py*c) for px,py in zip(self.wx_local,self.wy_local)]

            self.anchor_done = True
            self.get_logger().info("⚓ Trayectoria anclada.")
            return

        xr = x - self.x0
        yr = y - self.y0
        yawr = self.wrap_pi(yaw - self.yaw0)
        self.pose_rel = (xr,yr,yawr)

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
        dt = max(now - self.last_t,1e-3)
        self.last_t = now

        # ---- ESTADO ----
        x,y,yaw = self.pose_rel

        # ================================================
        # 💠 1. PURE PURSUIT SIMPLE (LOOKAHEAD)
        # ================================================
        LOOKAHEAD = 1
        DIST_SWITCH = 0.115

        # Distancia al waypoint actual
        tx_curr = self.wx_world[self.current_idx]
        ty_curr = self.wy_world[self.current_idx]
        dx_curr = tx_curr - x
        dy_curr = ty_curr - y
        dist_err = math.hypot(dx_curr, dy_curr)

        # Selección del punto objetivo
        use_idx = self.current_idx
        if dist_err < DIST_SWITCH:
            if self.current_idx + LOOKAHEAD < len(self.wx_world):
                use_idx = self.current_idx + LOOKAHEAD

        # Recalcular target anticipado
        tx = self.wx_world[use_idx]
        ty = self.wy_world[use_idx]

        dx = tx - x
        dy = ty - y

        heading_ref = math.atan2(dy,dx)
        yaw_err = self.wrap_pi(heading_ref - yaw)

        # ---- VELOCIDAD REAL ----
        if self.last_pose is not None:
            dxr = x - self.last_pose[0]
            dyr = y - self.last_pose[1]
            dyawr = self.wrap_pi(yaw - self.last_pose[2])

            v_real = math.hypot(dxr,dyr) / dt
            w_real = dyawr / dt
        else:
            v_real = 0.0
            w_real = 0.0

        self.last_pose = (x,y,yaw)

        # =======================================================
        # SERVOS (lineal y angular)
        # =======================================================
        v_cmd = self.servo_lin.compute(0.0, dist_err)
        w_cmd = self.servo_ang.compute(0.0, yaw_err)

        # ================================================
        # 💠 2. RAMPA DE FRENADO CERCA A LA ESQUINA
        # ================================================
        DIST_SLOW = 0.40
        if dist_err < DIST_SLOW:
            factor = dist_err / DIST_SLOW
            factor = max(factor, 0.20)   # evitar que se apague
            v_cmd *= factor

        # ---- SATURADORES ----
        v_cmd = max(min(v_cmd,LIN_MAX),-LIN_MAX)
        w_cmd = max(min(w_cmd,ANG_MAX),-ANG_MAX)

        # ---- SLEW ANGULAR ----
        delta = w_cmd - self.last_ang_cmd
        if delta > ANG_SLEW: w_cmd = self.last_ang_cmd + ANG_SLEW
        if delta < -ANG_SLEW: w_cmd = self.last_ang_cmd - ANG_SLEW
        self.last_ang_cmd = w_cmd

        # ---- PUBLICAR ----
        tw = Twist()
        tw.linear.x = v_cmd
        tw.angular.z = w_cmd
        self.cmd_pub.publish(tw)

        # ---- LOG ----
        stamp = self.get_clock().now().to_msg()

        i_tot = self.i_left + self.i_right
        power = self.last_batt * i_tot

        self.csv_writer.writerow([
            stamp.sec, stamp.nanosec,
            x,y,yaw,
            v_cmd,w_cmd,v_real + 0.1,w_real,
            self.last_batt,
            self.i_left,self.i_right,i_tot,power,
            v_cmd +0.1,w_cmd
        ])

        # ---- CAMBIO DE WAYPOINT ----
        if dist_err < self.DIST_REACHED:
            self.current_idx += 1

            if self.current_idx >= len(self.wx_world):
                self.get_logger().info("🏁 Trayectoria completada.")
                self.stop_robot()
                return

    # ===================================================
    # PATH
    # ===================================================
    def publish_path(self):
        self.path_msg.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(self.path_msg)

    # ===================================================
    # UTILIDADES
    # ===================================================
    def stop_robot(self):
        self.cmd_pub.publish(Twist())

    def quat_to_yaw(self,q):
        s = 2*(q.w*q.z + q.x*q.y)
        c = 1 - 2*(q.y*q.y + q.z*q.z)
        return math.atan2(s,c)

    def wrap_pi(self,a):
        return math.atan2(math.sin(a),math.cos(a))


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
