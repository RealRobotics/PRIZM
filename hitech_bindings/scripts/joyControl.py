#!/usr/bin/env python3

# from random import getstate

import rclpy
from rclpy.node import Node
import time
from prizm_interfaces.srv import SetHitechPosition
from sensor_msgs.msg import Joy


class JoyControl(Node):
    def __init__(self):
        super().__init__("JoyControl")
        self.service_caller = self.create_client(
            SetHitechPosition, "/set_position")

        self.joy_sub = self.create_subscription(
            Joy, "/joy", self.joy_callback, 10)

        self.positions = []

    def joy_callback(self, msg: Joy):
        # msg.buttons
        # self.get_logger().info(msg.buttons)
        if msg.axes[1]:
            self.positions[0] += int(msg.axes[1] * 100) / 100 * 512
        if msg.axes[2]:
            self.positions[3] += int(msg.axes[3] * 100) / 100 * 512

        self.call_service(self.positions)

    def call_service(self, positions: list[int]):
        req = SetHitechPosition.Request()
        req.hitech_position.size = 2
        req.hitech_position.motor_id = [0x43, 0x99]
        req.hitech_position.motor_pos = positions

        self.future = self.service_caller.call_async(req)
        # rclpy.spin_until_future_complete(self, self.future)
        while not self.future.done():   
            time.sleep(1e-3)
        return self.future.result()


def main(args=None):
    rclpy.init(args=args)

    joyControl = JoyControl()

    rclpy.spin(joyControl)

    rclpy.shutdown()

if __name__ == '__main__':
    main()