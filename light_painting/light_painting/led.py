import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from std_srvs.srv import Empty
from geometry_msgs.msg import Point, PoseStamped
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterDescriptor

import math


class LED(Node):

    def __init__(self):
        super().__init__("LED")
        self.declare_parameter("control", "radius", ParameterDescriptor(
            description="Type of controller for the LEDs (radius | line)"))
        self.declare_parameter("threshold", 0.03, ParameterDescriptor(
            descrition="Threshold distance for turning the LED off"))

        self.control = self.get_parameter(
            "control").get_parameter_value().string_value
        self.threshold = self.get_parameter(
            "threshold").get_parameter_value().double_value

        self.waypoint = Point()

        # LED bitmask requests for all LED's off
        params_off = Parameter()
        params_off.value.type = 2  # integer parameter
        params_off.name = 'cf231.params.led.bitmask'
        params_off.value.integer_value = int('0b10000000', base=0)
        self.off_req = SetParameters.Request()
        self.off_req.parameters = [params_off]

        # Turn just the blue right LED on
        params_blue = Parameter()
        params_blue.value.type = 2  # integer parameter
        params_blue.name = 'cf231.params.led.bitmask'
        params_blue.value.integer_value = int('0b10100000', base=0)
        self.blue_req = SetParameters.Request()
        self.blue_req.parameters = [params_blue]

        # Use a singular callback group to ensure services don't hang
        self.cb_group = MutuallyExclusiveCallbackGroup()

        # Service clients
        self.set_param = self.create_client(SetParameters,
                                            "/crazyflie_server/set_parameters",
                                            callback_group=self.cb_group)

        # Subscribers
        self.waypoint_sub = self.create_subscription(Point,
                                                     "waypoint",
                                                     self.waypoint_cb,
                                                     10)

        # State Subscriber changes depending on the control mode
        if (self.control == "radius"):
            self.pose_sub = self.create_subscription(PoseStamped,
                                                     "/cf231/pose",
                                                     self.radius_cb, 10)
        elif (self.control == "line"):
            self.pose_sub = self.create_subscription(PoseStamped,
                                                     "/cf231/pose",
                                                     self.line_cb, 10)
        else:
            self.get_logger().error("Control type invalid")
            raise NameError

    def waypoint_cb(self, msg):
        self.waypoint = msg

    def radius_cb(self, msg):
        current_pos = msg.pose.position
        dx = abs(current_pos.x-self.waypoint.x)
        dy = abs(current_pos.y-self.waypoint.y)
        dz = abs(current_pos.z-self.waypoint.z)

        if math.sqrt(dx**2 + dy**2 + dz**2) <= self.threshold:
            # turn LED on if close
            self.set_param(self.blue_req)
        else:
            # else turn off
            self.set_param(self.off_req)

    def line_cb(self, msg):
        current_pos = msg.pose.position

        pass


def main(args=None):
    rclpy.init(args=args)
    led = LED()
    rclpy.spin(led)
    rclpy.shutdown()
