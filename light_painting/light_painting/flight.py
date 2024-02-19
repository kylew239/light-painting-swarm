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
import asyncio
from time import sleep


class State(Enum):
    """Determine the states of the drone."""

    STOPPED = auto(),
    FLYING = auto(),
    TAKEOFF = auto(),
    PAINTING = auto(),
    STOPPING = auto(),
    HOMING = auto()


class Flight(Node):

    def __init__(self):
        super().__init__("flight")
        self.height = 0.30
        self.prev_state = State.STOPPED
        self.state = State.STOPPED
        self.prev_pose = Point()
        self.trajectory_id = 0
        self.pose_counter = 0       # TODO: remove counter

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
        req = UploadTrajectory.Request()
        req.trajectory_id = self.trajectory_id
        temp = TrajectoryPolynomialPiece()

        # Read csv and get data
        filename = "/home/kyle/winterProject/src/uav_trajectories/build/traj.csv"
        with open(filename, mode='r') as file:
            csvFile = csv.reader(file)
            data = list(csvFile)

        data.pop(0) # Get rid of heading
        for traj in data:
            # Get values from csv
            temp.duration = Duration(nanosec=(int(float(traj[0])*1e9)))
            temp.poly_x = [float(traj[i]) for i in range(1, 9)]
            temp.poly_y = [float(traj[i]) for i in range(9, 17)]
            temp.poly_z = [float(traj[i]) for i in range(17, 25)]
            temp.poly_yaw = [float(traj[i]) for i in range(25, 33)]


            # append to the trajectory
            req.pieces.append(temp)

        # TODO: Temporary geometry
        # x = [0, 1, 1, 0, 0, 1, 1, 0]
        # y = [-0.3, -0.3, 0.3, 0.3, -0.3, -0.3, 0.3, 0.3]
        # z = [0.3, 0.3, 0.3, 0.3, 0.9, 0.9, 0.9, 0.9]

        # # Relative
        # x = [0, 0, 0, 0, 0, 0, 0.1, 0]
        # y = [0, 0, 0, 0, 0, 0, 0.1, 0]
        # z = [0, 0, 0, 0, 0, 0, 0, 0]

        self.upload_trajectory.call_async(req)
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
        # Check if crazyflie has stopped moving and is ready for next command
        pos = msg.pose.position
        x_diff = abs(pos.x-self.prev_pose.x)
        y_diff = abs(pos.y-self.prev_pose.y)
        z_diff = abs(pos.z-self.prev_pose.z)
        self.prev_pose = pos
        if self.state == State.FLYING:
            if (x_diff < 0.01 and y_diff < 0.01 and z_diff < 0.01):
                if self.pose_counter > 30:
                    # Update state for next timer action
                    self.get_logger().error("Stopped moving")
                    if self.prev_state == State.TAKEOFF:
                        self.state = State.HOMING

                    if self.prev_state == State.PAINTING:
                        self.state = State.STOPPING
                    
                    if self.prev_state == State.HOMING:
                        self.state = State.PAINTING
                else:
                    self.pose_counter += 1
            else:
                self.pose_counter = 0
        else:
            self.pose_counter = 0

    async def timer_cb(self):
        if self.state == State.TAKEOFF:
            test = Duration(sec=1)
            takeoffReq = Takeoff.Request()
            takeoffReq.height = 0.3
            takeoffReq.duration = test
            self.get_logger().error("taking off")
            self.state = State.FLYING
            self.prev_state = State.TAKEOFF
            await self.cf_takeoff.call_async(takeoffReq)
            self.pose_counter = 0

        if self.state == State.HOMING:
            gotoReq = GoTo.Request()
            gotoReq.relative = False
            gotoReq.goal = Point(x=0.5, y=-0.5, z=0.3)
            gotoReq.duration = Duration(sec=1)
            self.state = State.FLYING
            self.prev_state = State.HOMING
            self.get_logger().error('flying home')
            await self.cf_goto.call_async(gotoReq)
            self.pose_counter = 0

        if self.state == State.PAINTING:
            # self.get_logger().error("starting shutter")
            # await self.shutter_start.call_async(Empty.Request())
            trajReq = StartTrajectory.Request()
            trajReq.trajectory_id = self.trajectory_id
            trajReq.relative = False
            trajReq.reversed = False
            trajReq.timescale = 1.0
            self.get_logger().error("starting path")
            self.state = State.FLYING
            self.prev_state = State.PAINTING
            await self.traj.call_async(trajReq)
            self.pose_counter = 0

        # if self.state == State.STOPPING:
        #     test = Duration(sec = 1)
        #     self.get_logger().error("stopping shutter")
        #     await self.shutter_stop.call_async(Empty.Request())
        #     landReq = Land.Request()
        #     landReq.height = 0.5
        #     landReq.duration = test
        #     self.get_logger().error('landing')
        #     self.state = State.STOPPED
        #     self.prev_state = State.PAINTING
        #     await self.cf_land.call_async(landReq)


def main(args=None):
    rclpy.init(args=args)
    flight = Flight()
    rclpy.spin(flight)
    rclpy.shutdown()
