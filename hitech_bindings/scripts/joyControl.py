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
            HitechPosition, '/position', 10)

        self.joy_sub = self.create_subscription(
            Joy, '/joy', self.joy_callback, 10)

        self.pos_1 = 0
        self.pos_2 = 0

    def joy_callback(self, msg: Joy):
        # msg.buttons
        # self.get_logger().info(msg.buttons)
        if msg.axes[1]:
            self.pos_1 += int(msg.axes[1]) * 100
        if msg.axes[2]:
            self.pos_2 += int(msg.axes[3]) * 100
        if self.pos_1 <= 0:
            self.pos_1 = 0

        if self.pos_2 <= 0:
            self.pos_2 = 0

        self.pub_positions()

    def pub_positions(self):
        msg = HitechPosition()
        msg.size = 2
        msg.motor_id = [0x43, 0x99]
        msg.motor_pos = [self.pos_1, self.pos_2]

        self.pos_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    joyControl = JoyControl()

    rclpy.spin(joyControl)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
