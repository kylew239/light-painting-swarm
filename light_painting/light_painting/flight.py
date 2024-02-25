import rclpy
from rclpy.node import Node
from enum import Enum, auto
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from crazyflie_interfaces.srv import Takeoff, Land, GoTo, StartTrajectory, UploadTrajectory
from std_srvs.srv import Empty
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point, PoseStamped
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter

import csv
import math
from time import sleep


class State(Enum):
    """Determine the states of the drone."""

    STOPPED = auto(),
    FLYING = auto(),
    TAKEOFF = auto(),
    CAMERA = auto(),
    STOPPING = auto(),
    HOMING = auto(),
    WAYPOINT = auto(),
    LANDING = auto()


class Flight(Node):

    def __init__(self):
        super().__init__("flight")
        self.height = 0.30
        self.prev_state = State.STOPPED
        self.state = State.STOPPED
        self.waypoints = []
        self.next_pos = Point()
        self.goal = Point()
        self.current_pos = Point()

        # Goto request
        self.gotoReq = GoTo.Request()
        self.gotoReq.relative = False
        self.gotoReq.duration = Duration(sec=1)

        # Use a singular callback group to ensure services don't hang
        self.cb_group = MutuallyExclusiveCallbackGroup()

        # Service clients
        # Crazyflie Server
        self.cf_takeoff = self.create_client(Takeoff,
                                             "/cf231/takeoff",
                                             callback_group=self.cb_group)
        self.cf_land = self.create_client(Land,
                                          "/cf231/land",
                                          callback_group=self.cb_group)
        self.cf_goto = self.create_client(GoTo,
                                          "/cf231/go_to",
                                          callback_group=self.cb_group)

        self.upload_trajectory = self.create_client(UploadTrajectory,
                                                    "/cf231/upload_trajectory",
                                                    callback_group=self.cb_group)

        self.traj = self.create_client(StartTrajectory,
                                       "/cf231/start_trajectory",
                                       callback_group=self.cb_group)

        self.set_param = self.create_client(SetParameters,
                                            "/crazyflie_server/set_parameters",
                                            callback_group=self.cb_group)

        # Camera
        self.shutter_start = self.create_client(Empty,
                                                "shutter_start",
                                                callback_group=self.cb_group)

        self.shutter_stop = self.create_client(Empty,
                                               "shutter_stop",
                                               callback_group=self.cb_group)

        # Service Servers
        self.start = self.create_service(Empty,
                                         "start",
                                         self.start_callback)

        # TODO:
        self.upload = self.create_service(Empty,
                                          "cube",
                                          self.upload_cb)

        # State Subscriber
        self.pose_sub = self.create_subscription(PoseStamped,
                                                 "/cf231/pose",
                                                 self.pose_cb, 10)

        # Timer
        self.timer = self.create_timer(1.0/10.0, self.timer_cb)

    def upload_cb(self, request, response):
        # Read csv and get data
        filename = "/home/kyle/winterProject/src/uav_trajectories/build/test.csv"
        with open(filename, mode='r') as file:
            csvFile = csv.reader(file)
            self.waypoints = list(csvFile)

        self.get_logger().info(
            f"Successfully saved {len(self.waypoints)} points")

        return response

    async def start_callback(self, request, response):
        # Turn off all LEDs except for the blue one
        params = Parameter()
        params.value.type = 2 # integer parameter
        params.value.integer_value = int('0b10100000', base=0)
        params.name = 'cf231.params.led.bitmask'

        req = SetParameters.Request()
        req.parameters = [params]
        await self.set_param.call_async(req)

        self.state = State.TAKEOFF
 
        return response

    def pose_cb(self, msg):
        self.current_pos = msg.pose.position
        # Check if crazyflie should be moving
        if self.state == State.FLYING:
            # Check if crazyflie is close enough to the goal
            if close(self.current_pos, self.goal):
                if self.prev_state == State.HOMING:
                    self.state = State.CAMERA

                if self.prev_state == State.WAYPOINT:
                    self.state = State.WAYPOINT
                
                if self.prev_state == State.STOPPING:
                    self.state = State.STOPPED

            # Takeoff doesn't maintain x and y distances
            # Need to calculate seperately
            if self.prev_state == State.TAKEOFF:
                if self.current_pos.z >= 0.3:  # Takeoff Height TODO: Make into parameter
                    self.state = State.HOMING

    async def timer_cb(self):
        if self.state == State.TAKEOFF:
            self.get_logger().info("taking off")
            self.state = State.FLYING
            self.prev_state = State.TAKEOFF

            takeoffReq = Takeoff.Request()
            takeoffReq.height = 0.3
            takeoffReq.duration = Duration(sec=1)

            self.goal = self.current_pos
            self.goal.z = self.goal.z + 0.3

            await self.cf_takeoff.call_async(takeoffReq)
            self.pose_counter = 0

        if self.state == State.HOMING:
            self.get_logger().info('flying to first point')
            self.state = State.FLYING
            self.prev_state = State.HOMING

            # Fly to goal
            self.gotoReq.goal = self.goal
            await self.cf_goto.call_async(self.gotoReq)
            self.pose_counter = 0

        if self.state == State.CAMERA:
            self.state = State.WAYPOINT
            self.prev_state = State.CAMERA

            # Start Camera
            self.get_logger().info("starting shutter")
            await self.shutter_start.call_async(Empty.Request())

        if self.state == State.WAYPOINT:
            self.get_logger().info(f"Points left: {len(self.waypoints)}")
            self.state = State.FLYING
            self.prev_state = State.WAYPOINT

            # If there are waypoints left, continue navigating
            if len(self.waypoints) > 0:
                point = self.waypoints.pop(0)
                self.goal = Point(
                    x=float(point[0]),
                    y=float(point[1]),
                    z=float(point[2])
                )

                self.gotoReq.goal = self.goal
                await self.cf_goto.call_async(self.gotoReq)
                self.pose_counter = 0

            # No waypoints left, stopping
            else:
                self.state = State.STOPPING

        if self.state == State.STOPPING:
            self.state = State.STOPPED
            self.prev_state = State.CAMERA
            self.get_logger().info("stopping shutter")
            await self.shutter_stop.call_async(Empty.Request())

            # fly 15cm above the ground, helps with a smoother landing
            self.gotoReq.duration = Duration(sec=3)
            self.gotoReq.goal.z = 0.15
            self.state = State.FLYING
            self.prev_state = State.STOPPING
            await self.cf_goto.call_async(self.gotoReq)

        if self.state == State.LANDING:
            self.get_logger().info('landing')
            landReq = Land.Request()
            landReq.height = 0.3
            landReq.duration = Duration(sec=1)
            self.state = State.STOPPED
            await self.cf_land.call_async(landReq)


def main(args=None):
    rclpy.init(args=args)
    flight = Flight()
    rclpy.spin(flight)
    rclpy.shutdown()


def close(p1: Point,
          p2: Point,
          threshold: float = 0.015,
          absolute: bool = False) -> bool:
    dx = abs(p1.x-p2.x)
    dy = abs(p1.y-p2.y)
    dz = abs(p1.z-p2.z)

    if absolute:
        return ((dx <= threshold) and (dy <= threshold) and (dz <= threshold))
    else:
        return (math.sqrt(dx**2 + dy**2 + dz**2) <= threshold)
