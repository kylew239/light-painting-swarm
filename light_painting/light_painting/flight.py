import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from crazyflie_interfaces.srv import Takeoff, Land, GoTo
from std_srvs.srv import Empty
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point
from rclpy.parameter import Parameter
import rclpy

from time import sleep

class Flight(Node):

    def __init__(self):
        super().__init__("flight")
        self.height = 0.30

        self.declare_parameter('cf231.params.led.bitmask', 128)

        # Services
        self.cf_takeoff = self.create_client(Takeoff,
                                        "/cf231/takeoff",
                                        callback_group=ReentrantCallbackGroup())
        self.cf_land = self.create_client(Land,
                                     "/cf231/land",
                                     callback_group=ReentrantCallbackGroup())
        self.cf_goto = self.create_client(GoTo,
                                     "/cf231/go_to",
                                     callback_group=ReentrantCallbackGroup())
        
        self.start = self.create_service(Empty,
                                    "start",
                                    self.start_callback)


        # DOES NOT WORK
        # These create a local parameter and set that
        # Need to set through launchfile
        # 128 = all off
        # 255 = all on
        self.params_off = Parameter(
            'cf231.params.led.bitmask',
            rclpy.Parameter.Type.INTEGER,
            128
        )
        self.params_on = Parameter(
            'cf231.params.led.bitmask',
            rclpy.Parameter.Type.INTEGER,
            256
        )
        
        
    async def start_callback(self, request, response):

        all_new_parameters = [self.params_off]
        self.set_parameters(all_new_parameters)


        test = Duration(sec = 1)
        # Take off
        takeoffReq = Takeoff.Request()
        takeoffReq.height = 0.3
        takeoffReq.duration = test
        await self.cf_takeoff.call_async(takeoffReq)
        sleep(1)

        # flying 0,5m forward
        test = Duration(sec = 3)
        gotoReq = GoTo.Request()
        gotoReq.relative = True
        gotoReq.goal = Point(x=0.96)
        gotoReq.duration = test
        await self.cf_goto.call_async(gotoReq)
        sleep(3)

        # fly 30cm to the right
        gotoReq.goal = Point(x=0.0, y=-0.55)
        await self.cf_goto.call_async(gotoReq)
        sleep(3)

        # fly 20cm back
        gotoReq.goal = Point(x=-0.96, y=0.0)
        await self.cf_goto.call_async(gotoReq)
        sleep(3)

        # fly back to start position
        gotoReq.goal = Point(x=0.0, y=0.55)
        await self.cf_goto.call_async(gotoReq)
        sleep(3)

        # Land
        landReq = Land.Request()
        landReq.height = 0.3
        landReq.duration = test
        await self.cf_land.call_async(landReq)

        all_new_parameters = [self.params_on]
        self.set_parameters(all_new_parameters)

        return response



def main(args=None):
    rclpy.init(args=args)
    flight = Flight()
    rclpy.spin(flight)
    rclpy.shutdown()