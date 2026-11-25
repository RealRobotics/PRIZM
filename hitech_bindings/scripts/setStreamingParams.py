#!/usr/bin/env python3

from prizm_interfaces.msg import StreamSettings
from prizm_interfaces.srv import SetStreamRegisters
import rclpy
from rclpy.node import Node


class SetStream(Node):
    def __init__(self):
        super().__init__('set_streaming_params')
        self.service_caller = self.create_client(
            SetStreamRegisters, '/set_stream_params'
        )
        while not self.service_caller.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not avaialble, waiting for 1s')

    def call_service(self):
        req = SetStreamRegisters.Request()
        stream_settings1 = StreamSettings()
        stream_settings1.servo_id = 0x42
        stream_settings1.stream_time = 10010  # 1Hz
        stream_settings1.toggle = True
        stream_settings1.addr_0 = [0x0C, 0x18]  # Position and Turn count
        stream_settings1.addr_1 = [0x10, 0x12]  # Torque and Voltage

        stream_settings2 = StreamSettings()
        stream_settings2.servo_id = 0x63
        stream_settings2.toggle = True
        stream_settings2.stream_time = 10010  # 10Hz
        stream_settings2.addr_0 = [0x0C, 0x18]  # Position and Turn count
        stream_settings2.addr_1 = [0x10, 0x12]  # Torque and Voltage

        req.stream_settings = [stream_settings1, stream_settings2]

        self.future = self.service_caller.call_async(req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):
    rclpy.init(args=args)

    setStream = SetStream()

    setStream.call_service()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
