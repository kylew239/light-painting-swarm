import rclpy
from rclpy.node import Node
from enum import Enum, auto
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from crazyflie_interfaces.srv import Takeoff, Land, GoTo, StartTrajectory, UploadTrajectory
from crazyflie_interfaces.msg import TrajectoryPolynomialPiece
from std_srvs.srv import Empty
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point, PoseStamped
from rclpy.parameter import Parameter
import rclpy

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
    WAYPOINT = auto()


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

        # # State Subscriber
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

        self.get_logger().error(f"Successfully saved {len(self.waypoints)} points")

        return response

    async def start_callback(self, request, response):
        self.state = State.TAKEOFF
        # emp = Empty.Request()

        # test = Duration(sec = 1)
        # # Take off
        # takeoffReq = Takeoff.Request()
        # takeoffReq.height = 0.5
        # takeoffReq.duration = test
        # self.state = State.MOVING
        # self.get_logger().error("taking off")
        # await self.cf_takeoff.call_async(takeoffReq)
        # sleep(2) # check pose

        # await self.shutter_start.call_async(emp)

        # # flying 0,5m forward
        # test = Duration(sec = 3)
        # gotoReq = GoTo.Request()
        # gotoReq.relative = True
        # gotoReq.goal = Point(x=1.0)
        # gotoReq.duration = test
        # self.state = State.MOVING
        # self.get_logger().error('flying to point 1')
        # await self.cf_goto.call_async(gotoReq)
        # sleep(3)

        # # fly 30cm to the right
        # gotoReq.goal = Point(x=0.0, z=0.20)
        # self.state = State.MOVING
        # self.get_logger().error('flying to point 2')
        # await self.cf_goto.call_async(gotoReq)
        # sleep(3)

        # # fly 20cm back
        # gotoReq.goal = Point(x=-1.0, y=0.0)
        # self.state = State.MOVING
        # self.get_logger().error('flying to point 3')
        # await self.cf_goto.call_async(gotoReq)
        # sleep(3)

        # # fly back to start position, but 20cm higher
        # gotoReq.goal = Point(x=0.0, z=-0.20)
        # self.state = State.MOVING
        # self.get_logger().error('flying to point 4')
        # await self.cf_goto.call_async(gotoReq)
        # sleep(3)

        # await self.shutter_stop.call_async(emp)

        # Land
        # landReq = Land.Request()
        # landReq.height = 0.5
        # landReq.duration = test
        # self.state = State.MOVING
        # self.get_logger().error('landing')
        # await self.cf_land.call_async(landReq)

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
            # else:
            #     # self.get_logger().error(f"Distance: {b}")
            #     # self.get_logger().error(f"C: {self.current_pos} ||| G: {self.goal}")

            # Takeoff doesn't maintain x and y distances
            # Need to calculate seperately
            if self.prev_state == State.TAKEOFF:
                if self.current_pos.z >= 0.3:  # Takeoff Height TODO: Make into parameter
                    self.state = State.HOMING

    async def timer_cb(self):
        # self.get_logger().error(f"Current state: {self.state} and previous state: {self.prev_state}")
        if self.state == State.TAKEOFF:
            self.get_logger().error("taking off")
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
            self.get_logger().error('flying to first point')
            self.state = State.FLYING
            self.prev_state = State.HOMING

            # Fly to goal
            self.gotoReq.goal = self.goal
            await self.cf_goto.call_async(self.gotoReq)
            self.pose_counter = 0

        if self.state == State.CAMERA:
            self.get_logger().error("starting Camera")
            self.state = State.WAYPOINT
            self.prev_state = State.CAMERA

            # Start Camera
            self.get_logger().error("starting shutter")
            await self.shutter_start.call_async(Empty.Request())

        if self.state == State.WAYPOINT:
            self.get_logger().error("moving to next point")
            self.get_logger().error(f"Points left: {len(self.waypoints)}")
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
            self.get_logger().error("stopping shutter")
            await self.shutter_stop.call_async(Empty.Request())

            self.get_logger().error('landing')
            landReq = Land.Request()
            landReq.height = 0.3
            landReq.duration = Duration(sec = 1)
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
