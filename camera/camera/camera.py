"""
Controls the camera shutter

Parameters:
    - arduino_port: port that the arduino is connected to
    - arduino_pin: pin on the arduino that the camera is connected to

Services:
    - shutter_start (std_msgs/Empty): service to start the shutter
    - shutter_stop (std_msgs/Empty): service to stop the shutter

"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rcl_interfaces.msg import ParameterDescriptor

from std_srvs.srv import Empty
from pyfirmata import Arduino


class Camera(Node):

    def __init__(self):
        super().__init__("camera")

        # Parameters
        self.declare_parameter("arduino_port", '/dev/ttyACM0', ParameterDescriptor(
            description="Port that the Arduinon is connected to"))
        self.declare_parameter("arduino_pin", 8, ParameterDescriptor(
            description="Pin on the Arduino that the camera is connected to"))

        self.port = self.get_parameter(
            "arduino_port").get_parameter_value().string_value
        self.pin = self.get_parameter(
            "arduino_pin").get_parameter_value().integer_value

        # counter used to keep track of multiple triggers
        self.count = 0

        # Arduino board setup
        self.board = Arduino(self.port)
        self.board.digital[self.pin].write(1)

        # Services
        self.shutter_start = self.create_service(Empty,
                                                 "shutter_start",
                                                 self.start_cb,
                                                 callback_group=ReentrantCallbackGroup())

        self.shutter_stop = self.create_service(Empty,
                                                "shutter_stop",
                                                self.stop_cb,
                                                callback_group=ReentrantCallbackGroup())

    async def start_cb(self, request, response):
        """Start the Camera shutter."""
        self.board.digital[self.pin].write(0)
        self.count += 1
        return response

    async def stop_cb(self, request, response):
        """Stop the Camera shutter."""
        self.count -= 1

        # Only stop the shutter once all the stop_shutters have been called
        if self.count <= 0:
            self.board.digital[self.pin].write(1)

        return response


def main(args=None):
    rclpy.init(args=args)
    camera = Camera()
    rclpy.spin(camera)
    rclpy.shutdown()
