#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Nodo: test_encoder_turn
- Avanza lentamente una vuelta (~2570 ticks).
- Lee encoders desde /sensors/core (mensaje SensorState).
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from kobuki_ros_interfaces.msg import SensorState


class TestEncoderTurn(Node):
    def __init__(self):
        super().__init__('test_encoder_turn')

        # Publicador de velocidad
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Suscripción correcta (ROS 2 Humble)
        self.create_subscription(SensorState, '/sensors/core', self.sensor_cb, 10)

        # Variables
        self.left_init = None
        self.right_init = None
        self.left_now = 0
        self.right_now = 0
        self.running = False
        self.target_ticks = 2570  # ≈ 1 vuelta de rueda

        # Timer de control
        self.timer = self.create_timer(0.02, self.timer_cb)
        self.get_logger().info("🚀 Nodo iniciado: el robot avanzará una vuelta y mostrará los ticks.")

    def sensor_cb(self, msg):
        if self.left_init is None:
            self.left_init = msg.left_encoder
            self.right_init = msg.right_encoder
            self.running = True
        self.left_now = msg.left_encoder
        self.right_now = msg.right_encoder

    def timer_cb(self):
        if not self.running or self.left_init is None:
            return

        # Diferencias (manejo de rollover de 16 bits)
        dl = (self.left_now - self.left_init) % 65536
        dr = (self.right_now - self.right_init) % 65536
        avg = (dl + dr) / 2.0

        self.get_logger().info(f"Ticks L={dl:.0f}, R={dr:.0f}, Prom={avg:.0f}")

        # Control de avance
        cmd = Twist()
        if avg < self.target_ticks:
            cmd.linear.x = 0.05  # velocidad lenta
        else:
            cmd.linear.x = 0.0
            self.running = False
            self.get_logger().info("✅ ¡Una vuelta completa detectada")
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = TestEncoderTurn()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Nodo detenido por el usuario.")
    finally:
        try:
            node.cmd_pub.publish(Twist())
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
