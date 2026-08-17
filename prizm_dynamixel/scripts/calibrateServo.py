#!/usr/bin/env python3

from numpy import source

import rclpy
from rclpy.node import Node

from prizm_interfaces.msg import DynamixelPos, DynamixelPosArr
import rclpy.time
from sensor_msgs.msg import Joy

import yaml, datetime
from pathlib import Path

class CalibrateServo(Node):

    # parameters
    # what servo id to use
    # where to save the calibration file

    def __init__(self):
        super().__init__("CalibrateServo")
        self.publish_pos = self.create_publisher(
            DynamixelPosArr, "servo_pos", 10)
        # self.joy_subscriber = self.create_subscription(Joy, "joy_input", self.joyCallack, 10)

        self.declare_parameter("servo_id", 0)
        self.declare_parameter(
            "output_path", "src/PRIZM/prizm_dynamixel/config/calibration.yaml")

        self.servo_id = self.get_parameter(
            "servo_id").get_parameter_value().integer_value
        self.curr_pos = 0.0
        self.direction = 1

        self.max_pos = 0.0
        self.zero_pos = 0.0
        self.min_pos = 0.0

        self.save_max = True
        self.save_zero = False
        self.save_min = False

    def initPos(self):
        # self.get_logger().info(f"Initialising position for id {self.servo_id}")
        # bring the servo to pos 0 so we have a define start point
        initial_pos = DynamixelPosArr()
        pos = DynamixelPos()
        pos.id = self.servo_id
        pos.position = self.curr_pos
        initial_pos.pos_array.append(pos)

        self.publish_pos.publish(
            initial_pos
        )

    def updatePos(self, pos: float):
        pos_arr = DynamixelPosArr()
        pos_ = DynamixelPos()
        pos_.id = self.servo_id
        pos_.position = pos
        pos_arr.pos_array.append(pos_)
        self.publish_pos.publish(pos_arr)

    def savePosition(self):
        if (self.save_max and not self.save_zero and not self.save_min):
            self.max_pos = self.curr_pos
            self.save_max = False
            self.save_zero = True
            self.save_min = False

        if (self.save_zero and not self.save_max and not self.save_min):
            self.zero_pos = self.curr_pos
            self.save_zero = False
            self.save_min = True

        if (self.save_min and not self.save_zero and not self.save_max):
            self.min_pos = self.curr_pos
            self.save_min = False

    def joyCallack(self, msg: Joy):
        # look here for the schema
        # https://docs.ros.org/en/ros2_packages/jazzy/api/joy/index.html

        # for now this will just use the face buttons
        # idx 0: A (X on PS): +1
        # idx 1: B (Circle on PS): +5
        # idx 2: X (Square on PS): +10
        # idx 3: Y (Triangle on PS): +20
        # idx 9: LB (L1 on PS): Reverse Direction
        # idx 10: RB (R1 on PS): Save Position
        buttons = msg.buttons
        if (buttons[0] == 1):
            self.curr_pos += 1 * self.direction
        if (buttons[1] == 1):
            self.curr_pos += 5 * self.direction
        if (buttons[2] == 1):
            self.curr_pos += 10 * self.direction
        if (buttons[3] == 1):
            self.curr_pos += 20 * self.direction
        if (buttons[9] == 1):
            self.direction *= -1
        if (buttons[10] == 1):
            self.savePosition()

    def dump_info(self):
        now = str(datetime.datetime.now())
        output_data = {
            "servo_config" : {
                {"date": now},
                {"id" : self.servo_id},
                {"max" : self.max_pos},
                {"min" : self.min_pos},
                {"zero" : self.zero_pos},
            }
        }
        # with open()
        # source_path = Path(__file__).resolve()
        # source_dir = source_path.parent
        # path = source_dir.replace("")
        # with open()
        path = "~/prizm/ros2_ws/PRIZM/prizm_dynamixel/config/"
        filename = "calibration_" + now + ".yaml"
        with open(path+filename, 'w') as outfile:
            yaml.dump(output_data, outfile, default_flow_style=False)

def main(args=None):
    rclpy.init(args=args)
    calibrate_servo = CalibrateServo()
    calibrate_servo.initPos()

    rclpy.spin(calibrate_servo)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
