#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
pid_autotune_node.py  —  Safe Autotune (P-sweep | Relay Åström–Hägglund)
- angular | linear
- Métodos: autotune_method := 'relay' (por defecto) | 'p_sweep'
- Guarda:
   1) autotune_summary_<modo>_<fecha>.csv  (Ku, Tu, Kp, Ki, Kd por ZN y TL)
   2) autotune_data_<modo>_<fecha>.csv     (time, error, output, measured, current_kp/method_state)

RECOMENDADO: usar 'relay' en angular. Forza oscilación suave y segura sin “latigazos”.
"""

import os, csv, time, math, statistics
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# --- PID mínimo para modo p_sweep (no se usa en relay) ---
class PIDController:
    def __init__(self, kp=0.0, ki=0.0, kd=0.0):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.i = 0.0
        self.e_prev = 0.0
    def compute(self, e, dt):
        self.i += e*dt
        d = (e - self.e_prev)/max(dt,1e-3)
        self.e_prev = e
        return self.kp*e + self.ki*self.i + self.kd*d

class PIDAutotune(Node):
    def __init__(self):
        super().__init__('pid_autotune_node')

        # -------- Parámetros --------
        self.declare_parameter('odom_topic', '/odom_raw')
        self.declare_parameter('mode', 'angular')          # angular | linear
        self.declare_parameter('autotune_method', 'relay') # relay | p_sweep
        self.declare_parameter('csv_dir', '/ros2_ws/ros2host_ws/src/turtlebot_scripts/turtlebot_scripts/')
        self.declare_parameter('test_duration', 45.0)

        # p_sweep
        self.declare_parameter('max_kp', 4.0)
        self.declare_parameter('kp_step', 0.2)
        self.declare_parameter('ref_value', 0.0)

        # relay (Åström–Hägglund)
        self.declare_parameter('relay_h', 0.5)   # amplitud de control (rad/s angular, m/s lineal)
        self.declare_parameter('relay_eps', 0.05) # histéresis en error
        self.declare_parameter('min_cycles', 3)   # ciclos completos requeridos

        # límites
        self.declare_parameter('w_limit', 1.5)   # rad/s
        self.declare_parameter('v_limit', 0.10)  # m/s

        # -------- Carga --------
        self.odom_topic      = self.get_parameter('odom_topic').value
        self.mode            = self.get_parameter('mode').value
        self.method          = self.get_parameter('autotune_method').value
        self.csv_dir         = self.get_parameter('csv_dir').value
        self.test_duration   = float(self.get_parameter('test_duration').value)
        self.max_kp          = float(self.get_parameter('max_kp').value)
        self.kp_step         = float(self.get_parameter('kp_step').value)
        self.ref_value       = float(self.get_parameter('ref_value').value)
        self.relay_h         = float(self.get_parameter('relay_h').value)
        self.relay_eps       = float(self.get_parameter('relay_eps').value)
        self.min_cycles      = int(self.get_parameter('min_cycles').value)
        self.w_limit         = float(self.get_parameter('w_limit').value)
        self.v_limit         = float(self.get_parameter('v_limit').value)

        if self.method == 'relay' and self.mode == 'angular' and abs(self.ref_value) < 0.05:
            self.ref_value = 0.0  # en relay la referencia 0 funciona perfecto

        # -------- Archivos CSV --------
        fecha = time.strftime("%Y-%m-%d_%H-%M-%S", time.localtime())
        os.makedirs(self.csv_dir, exist_ok=True)
        self.csv_summary = os.path.join(self.csv_dir, f"autotune_summary_{self.mode}_{fecha}.csv")
        self.csv_data    = os.path.join(self.csv_dir, f"autotune_data_{self.mode}_{fecha}.csv")
        self.sum_writer  = csv.writer(open(self.csv_summary, 'w', newline=''))
        self.sum_writer.writerow(['mode','Ku','Tu','Kp','Ki','Kd','method'])
        self.data_f = open(self.csv_data, 'w', newline='')
        self.data_writer = csv.writer(self.data_f)
        # nota: column5 cambia según método
        self.data_writer.writerow(['time','error','output','measured','note'])

        self.get_logger().info(f"📁 Summary → {self.csv_summary}")
        self.get_logger().info(f"📈 Data    → {self.csv_data}")

        # -------- ROS I/O --------
        self.create_subscription(Odometry, self.odom_topic, self.odom_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # -------- Estados internos --------
        self.dt = 0.05
        self.t0 = time.time()
        self.value = 0.0  # yaw (rad) o x (m)
        self.pid = PIDController()
        self.current_kp = 0.2

        # relay state
        self.relay_state = +1.0
        self.last_u = 0.0
        self.alpha = 0.2          # rampa (suavizado)
        self.peak_times = []      # tiempos de picos de salida
        self.peak_values = []     # amplitudes de salida
        self.last_sign = 0
        self.cross_count = 0

        # p_sweep detectores
        self.zero_cross_times = []
        self.periods = []
        self.osc_detected = False

        self.timer = self.create_timer(self.dt, self.loop)
        self.get_logger().info(f"🚀 Iniciando autotune [{self.method}] modo={self.mode}")

    # ------------- ODOM -------------
    def odom_cb(self, msg: Odometry):
        if self.mode == 'angular':
            q = msg.pose.pose.orientation
            self.value = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        else:
            self.value = msg.pose.pose.position.x

    # ------------- MAIN LOOP -------------
    def loop(self):
        t = time.time() - self.t0
        if t > self.test_duration:
            self.finish()
            return

        error = self.ref_value - self.value

        if self.method == 'relay':
            u_raw, note = self.relay_control(error, t)
        else:
            u_raw, note = self.p_sweep_control(error, t)

        # rampa + saturación
        u = self.last_u + self.alpha*(u_raw - self.last_u)
        self.last_u = u
        cmd = Twist()
        if self.mode == 'angular':
            cmd.angular.z = max(min(u, self.w_limit), -self.w_limit)
        else:
            cmd.linear.x  = max(min(u, self.v_limit), -self.v_limit)
        self.cmd_pub.publish(cmd)

        # log
        self.data_writer.writerow([t, error, u, self.value, note])

    # ------------- RELAY METHOD -------------
    def relay_control(self, error, t):
        # histéresis en el error
        if error >  self.relay_eps:
            self.relay_state = +1.0
        elif error < -self.relay_eps:
            self.relay_state = -1.0
        u_raw = self.relay_state * (self.relay_h if self.mode=='angular' else min(self.relay_h, self.v_limit))

        # detección de picos en la salida medida (value)
        sign = 1 if error >= 0 else -1
        if sign != self.last_sign and self.last_sign != 0:
            # cruce por cero del error: posible pico cerca
            # guardamos el pico máximo local reciente (últimos ~0.5 s)
            # aquí tomamos el valor instantáneo como proxy simple
            self.peak_times.append(t)
            self.peak_values.append(abs(self.value))
            # cuando haya al menos dos picos de mismo signo, estimamos Tu
            if len(self.peak_times) >= 3:
                Tu = (self.peak_times[-1] - self.peak_times[-3]) / 2.0
                a  = max(self.peak_values[-3:])  # amplitud aprox
                if a > 1e-3 and Tu > 0.2:
                    Ku = (4.0*self.relay_h) / (math.pi * a)
                    self.save_results(Ku, Tu)
        self.last_sign = sign
        return u_raw, f"relay_k:{self.relay_h}"

    # ------------- P-SWEEP METHOD -------------
    def p_sweep_control(self, error, t):
        self.pid.kp = self.current_kp
        u_raw = self.pid.compute(error, self.dt)
        # detectar oscilación por cruces de cero del error
        if len(self.zero_cross_times)==0:
            self.zero_cross_times.append(t)
        else:
            # cruce: cambio de signo
            # usamos valor guardado en compute (e_prev), así que miramos error actual vs último
            pass
        # cruce por cero simple
        # (para robustez, detectamos cuando error cambia de signo entre muestras)
        # guardamos los tiempos y calculamos período medio con varias muestras
        # Nota: en la práctica, el método relay es más confiable
        if hasattr(self, 'last_error'):
            if self.last_error * error < 0:
                self.zero_cross_times.append(t)
                if len(self.zero_cross_times) >= 3:
                    Tu = 2.0 * (self.zero_cross_times[-1] - self.zero_cross_times[-2])
                    # oscilación "aceptable"
                    if Tu > 0.2:
                        Ku = self.current_kp
                        self.save_results(Ku, Tu)
                        self.current_kp += self.kp_step
                        if self.current_kp > self.max_kp:
                            self.finish()
                        else:
                            self.reset_psweep()
        self.last_error = error
        return u_raw, f"kp:{self.current_kp:.2f}"

    # ------------- SAVE AND UTIL -------------
    def save_results(self, Ku, Tu):
        # Ziegler–Nichols PID
        Kp_zn = 0.6 * Ku
        Ki_zn = 1.2 * Kp_zn / Tu
        Kd_zn = 0.075 * Kp_zn * Tu
        self.sum_writer.writerow([self.mode, Ku, Tu, Kp_zn, Ki_zn, Kd_zn, 'Ziegler-Nichols'])

        # Tyreus–Luyben PID
        Kp_tl = 0.454 * Ku
        Ki_tl = Kp_tl / (2.2 * Tu)
        Kd_tl = Kp_tl * Tu / 6.3
        self.sum_writer.writerow([self.mode, Ku, Tu, Kp_tl, Ki_tl, Kd_tl, 'Tyreus-Luyben'])

        self.get_logger().info(f"🌀 Ku={Ku:.3f} Tu={Tu:.3f}s → ZN/TL guardados")

    def reset_psweep(self):
        self.zero_cross_times.clear()
        self.pid.i = 0.0
        self.pid.e_prev = 0.0
        self.get_logger().info(f"➡️  p_sweep: probando Kp={self.current_kp:.2f}")

    def finish(self):
        self.get_logger().info("✅ Autotune finalizado. Deteniendo robot.")
        self.cmd_pub.publish(Twist())
        self.data_f.flush(); self.data_f.close()
        self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = PIDAutotune()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Interrumpido manualmente.")
    finally:
        node.cmd_pub.publish(Twist())
        node.data_f.close()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
