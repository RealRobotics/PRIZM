#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from prizm_interfaces.msg import DynamixelPosArr
from prizm_interfaces.msg import DynamixelPos

class PosArrPub(Node):
    def __init__(self):
        super().__init__("pos_arr_pub")
        self.publisher = self.create_publisher(DynamixelPosArr, "servo_pos", 10)

    def publish_positions(self, ids: list, values: list):
        pos_arr = DynamixelPosArr()
        # for
        for id, value in zip(ids, values):
            position = DynamixelPos()
            position.id = id
            position.position = value
            print(id, value)
            pos_arr.pos_array.append(position)
        self.publisher.publish(pos_arr)

def main(args=None):
    rclpy.init(args=args)

    pos_arr_pub = PosArrPub()

    ids = [0]
    values = [69.0]

    pos_arr_pub.publish_positions(ids, values)


    rclpy.shutdown()

if __name__ == '__main__':
    main()