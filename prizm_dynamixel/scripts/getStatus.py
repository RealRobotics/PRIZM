#!/usr/bin/env python3

from random import getstate

import rclpy
from rclpy.node import Node

from prizm_interfaces.srv import DynamixelGetStatus
from prizm_interfaces.msg import DynamixelPosArr, DynamixelVelArr
from prizm_interfaces.msg import DynamixelPos, DynamixelVel


class GetStatus(Node):
    def __init__(self):
        super().__init__("getStatus")
        # self.publisher = self.create_publisher(DynamixelPosArr, "servo_pos", 10)
        self.service_caller = self.create_client(
            DynamixelGetStatus, "/motor_driver/get_status")
        self.ids = 0
        while not self.service_caller.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

    def call_service(self, ids):
        req = DynamixelGetStatus.Request()
        req.id = ids
        self.ids = ids
        self.future = self.service_caller.call_async(req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def print_results(self, response: DynamixelGetStatus.Response):
        pArr = DynamixelPosArr()
        vArr = DynamixelVelArr()
        pos = DynamixelPos()
        vel = DynamixelVel()

        pArr = response.pos_array
        vArr = response.vel_array

        positions = []
        velocities = []

        for pos in pArr.pos_array:
            positions.append(pos.position)
        for vel in vArr.vel_array:
            velocities.append(vel.velocity)

        for i in self.ids:
            self.get_logger().info('id: %d, torque %d op mode: %d position: %d, velocity: %d' %
                                   (i, response.operating_mode[i], response.torque_enable[i], 
                                    positions[i], velocities[i]))


def main(args=None):
    rclpy.init(args=args)

    getStatus = GetStatus()

    ids = [0]

    response = getStatus.call_service(ids)
    getStatus.print_results(response)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
