#!/usr/bin/env python3

import rclpy
from rclpy.node import Node


from prizm_interfaces.msg import DynamixelVelArr
from prizm_interfaces.msg import DynamixelVel

class PosArrPub(Node):
    def __init__(self):
        super().__init__("pos_arr_pub")
        self.publisher = self.create_publisher(DynamixelVelArr, "motor_vel", 10)

    def publish_positions(self, ids: list, values: list):
        pos_arr = DynamixelVelArr()
        # for
        for id, value in zip(ids, values):
            position = DynamixelVel()
            position.id = id
            position.velocity = value
            print(id, value)
            pos_arr.vel_array.append(position)
        self.publisher.publish(pos_arr)

def main(args=None):
    rclpy.init(args=args)

    pos_arr_pub = PosArrPub()

    ids = [0, 1]
    values = [20.0, 60.]

    pos_arr_pub.publish_positions(ids, values)


    rclpy.shutdown()

if __name__ == '__main__':
    main()