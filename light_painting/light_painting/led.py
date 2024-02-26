import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from std_srvs.srv import Empty
from geometry_msgs.msg import Point, PoseStamped, Vector3
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterDescriptor

from enum import Enum, auto
import math


class State(Enum):
    """Record the state of the LED."""
    ON = auto(),
    OFF = auto(),


class LED(Node):

    def __init__(self):
        super().__init__("LED")
        self.declare_parameter("control", "radius", ParameterDescriptor(
            description="Type of controller for the LEDs (radius | line)"))
        self.declare_parameter("threshold", 0.05, ParameterDescriptor(
            descrition="Threshold distance for turning the LED off"))

        self.control = self.get_parameter(
            "control").get_parameter_value().string_value
        self.threshold = self.get_parameter(
            "threshold").get_parameter_value().double_value

        self.waypoint = Point()
        self.prev_waypoint = Point()
        self.line = Vector3()
        self.toggle = False
        self.state = State.OFF

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

        # Service Server
        self.toggle = self.create_service(Empty,
                                          "toggle_led",
                                          self.toggle_cb)

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

    def toggle_cb(self, request, response):
        self.toggle = not self.toggle
        return response

    def waypoint_cb(self, msg):
        # for line
        # update direction line
        self.line = calc_vector(msg, self.waypoint)
        mag = get_mag(self.line)

        # Normalize line vector
        self.line = Vector3(
            x=self.line.x / mag,
            y=self.line.y / mag,
            z=self.line.z / mag
        )

        # update waypoint storage
        self.prev_waypoint = self.waypoint
        self.waypoint = msg

    async def radius_cb(self, msg):
        current_pos = msg.pose.position
        dx = abs(current_pos.x-self.waypoint.x)
        dy = abs(current_pos.y-self.waypoint.y)
        dz = abs(current_pos.z-self.waypoint.z)

        if self.toggle:
            if (math.sqrt(dx**2 + dy**2 + dz**2) <= self.threshold) and (self.state == State.OFF):
                # turn LED on if close
                # Also check to make sure it was previously off
                await self.set_param.call_async(self.blue_req)
                self.state = State.ON
            elif self.state == State.ON:
                # else turn off
                await self.set_param.call_async(self.off_req)
                self.state = State.OFF

    async def line_cb(self, msg):
        current_pos = msg.pose.position

        # prev_goal to current point vector
        vec_to_curr = calc_vector(current_pos, self.prev_waypoint)

        # dot product
        dot = self.line.x * vec_to_curr.x + self.line.y * \
            vec_to_curr.y + self.line.z * vec_to_curr.z

        # project the current point vector onto line vector
        proj = Vector3(
            x=self.line.x * dot,
            y=self.line.y * dot,
            z=self.line.z * dot
        )

        # distance from line to point
        dist_vec = Vector3(
            x=vec_to_curr.x - proj.x,
            y=vec_to_curr.y - proj.y,
            z=vec_to_curr.z - proj.z
        )

        if self.toggle:
            if (get_mag(dist_vec) < self.threshold) and (self.state == State.OFF):
                # if distance to line is close enough, turn the LED on
                # Also check to make sure it was previously off
                await self.set_param.call_async(self.blue_req)
                self.state = State.ON
            elif self.state == State.ON:
                # else turn the LED off
                await self.set_param.call_async(self.off_req)
                self.state = State.OFF


def main(args=None):
    rclpy.init(args=args)
    led = LED()
    rclpy.spin(led)
    rclpy.shutdown()


def calc_vector(p1: Point, p2: Point) -> Vector3:
    """
    Calculates the vector between two points.

    Args:
        p1 (geometry_msgs/Point): The head of the vector
        p2 (geometry_msgs/Point): The tail of the vector

    Returns
    -------
        Vector3: A geometery_msgs/Vector3 bewteen the two points
    """
    return Vector3(
        x=p1.x - p2.x,
        y=p1.y - p2.y,
        z=p1.z - p2.z
    )


def get_mag(v: Vector3) -> float:
    """
    Calculates the magnitude of a vector

    Args:
        v (geometry_msgs/Vector3): The vector

    Returns
    -------
        float: The magnitude
    """
    return math.sqrt(v.x**2 + v.y**2 + v.z**2)
