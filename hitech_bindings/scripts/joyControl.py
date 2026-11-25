#!/usr/bin/env python3

# from random import getstate

from prizm_interfaces.msg import HitechPosition
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy


class JoyControl(Node):
    def __init__(self):
        super().__init__('JoyControl')
        self.pos_publisher = self.create_publisher(
            HitechPosition, '/position_input', 10)

        self.joy_sub = self.create_subscription(
            Joy, '/joy', self.joy_callback, 10)

        self.declare_parameter('publish_rate', 10.0)  # Hz
        rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.timer = self.create_timer(1.0 / rate, self.timer_callback)

        self.axis_1 = 0.0
        self.axis_2 = 0.0
        self.pos_1 = 0
        self.pos_2 = 0
        self.last_sent = None

    def joy_callback(self, msg: Joy):
        # only cache the latest stick values; publishing happens on the timer
        self.axis_1 = msg.axes[1]
        self.axis_2 = msg.axes[3]

    def timer_callback(self):
        self.pos_1 = max(0, self.pos_1 + int(self.axis_1 * 400))
        self.pos_2 = max(0, self.pos_2 + int(self.axis_2 * 400))
        if (self.pos_1, self.pos_2) != self.last_sent:
            self.pub_positions()
            self.last_sent = (self.pos_1, self.pos_2)

    def pub_positions(self):
        msg = HitechPosition()
        msg.size = 2
        msg.motor_id = [0x42, 0x63]
        msg.motor_pos = [self.pos_1, self.pos_2]

        self.pos_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    joyControl = JoyControl()

    rclpy.spin(joyControl)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
