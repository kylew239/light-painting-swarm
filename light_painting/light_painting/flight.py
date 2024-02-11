import rclpy
from rclpy.node import Node
from enum import Enum, auto
from rclpy.callback_groups import ReentrantCallbackGroup

from crazyflie_interfaces.srv import Takeoff, Land, GoTo, StartTrajectory, UploadTrajectory
from crazyflie_interfaces.msg import TrajectoryPolynomialPiece
from std_srvs.srv import Empty
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point, Pose
from rclpy.parameter import Parameter
import rclpy

import asyncio
from time import sleep

class State(Enum):
    """Determine the states of the drone."""

    STOPPED = auto(),
    FLYING = auto(),
    TAKEOFF = auto(),
    PAINTING = auto(),
    STOPPING = auto(),

class Flight(Node):

    def __init__(self):
        super().__init__("flight")
        self.height = 0.30
        self.prev_state = State.STOPPED
        self.state = State.STOPPED
        self.prev_pose = Pose()      
        self.trajectory_id = 0;  

        # Service clients
        self.cf_takeoff = self.create_client(Takeoff,
                                        "/cf231/takeoff",
                                        callback_group=ReentrantCallbackGroup())
        self.cf_land = self.create_client(Land,
                                     "/cf231/land",
                                     callback_group=ReentrantCallbackGroup())
        self.cf_goto = self.create_client(GoTo,
                                     "/cf231/go_to",
                                     callback_group=ReentrantCallbackGroup())
        
        self.upload_trajectory = self.create_client(UploadTrajectory,
                                          "/cf231/upload_trajectory",
                                          callback_group=ReentrantCallbackGroup())

        self.traj = self.create_client(StartTrajectory,
                                        "/cf231/start_trajectory",
                                        callback_group=ReentrantCallbackGroup())
        
        self.shutter_start = self.create_client(Empty,
                                                "shutter_start",
                                                callback_group=ReentrantCallbackGroup())
        
        self.shutter_stop = self.create_client(Empty,
                                               "shutter_stop",
                                               callback_group=ReentrantCallbackGroup())
        
        # Service Servers
        self.start = self.create_service(Empty,
                                    "start",
                                    self.start_callback)
        
        #TODO:
        self.upload = self.create_service(Empty,
                                          "cube",
                                          self.upload_cb)

        # State Subscriber
        self.pose_sub = self.create_subscription(Pose,
                                                 "/cf231/pose",
                                                 self.pose_cb, 10)
        
        # Timer
        self.timer = self.create_timer(1.0/200.0, self.timer_cb)
        

    def upload_cb(self, request, response):
        req = UploadTrajectory.Request()
        req.trajectory_id = self.trajectory_id

        # TODO: Temporary geometry
        x = [0, 1, 1, 0, 0, 1, 1, 0]
        y = [-0.5, -0.5, 0.5, 0.5, -0.5, -0.5, 0.5, 0.5]
        z = [0.3, 0.3, 0.3, 0.3, 1.3, 1.3, 1.3, 1.3]

        temp = TrajectoryPolynomialPiece()
        temp.poly_x = x
        temp.poly_y = y
        temp.poly_z = z
        temp.poly_yaw = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        req.pieces = [temp]

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
        pose = msg.pose.position
        x_diff = abs(pose.x-self.prev_pose.position.x)
        y_diff = abs(pose.y-self.prev_pose.position.y)
        z_diff = abs(pose.z-self.prev_pose.position.z)
        self.prev_pose = pose
        if self.state == State.FLYING:
            self.get_logger().error("not switching states")

        if(x_diff < 0.01 and y_diff < 0.01 and z_diff < 0.01):
            # Update state for next timer action
            self.get_logger().error("SWITCHING STATES")
            if self.prev_state == State.TAKEOFF:
                self.state = State.PAINTING
    
            if self.prev_state == State.PAINTING:
                self.state = State.STOPPING
        

    async def timer_cb(self):
        if self.state == State.TAKEOFF:
            test = Duration(sec = 1)
            takeoffReq = Takeoff.Request()
            takeoffReq.height = 0.3
            takeoffReq.duration = test
            self.get_logger().error("taking off")
            self.state = State.FLYING
            self.prev_state = State.TAKEOFF
            await self.cf_takeoff.call_async(takeoffReq)

            

        if self.state == State.PAINTING:
            self.get_logger().error("starting shutter")
            await self.shutter_start.call_async(Empty.Request())
            trajReq = StartTrajectory.Request()
            trajReq.trajectory_id = self.trajectory_id
            trajReq.relative = False
            trajReq.reversed = True
            self.get_logger().error("starting path")
            self.state = State.FLYING
            self.prev_state = State.PAINTING
            await self.traj.call_async(trajReq)

            

        if self.state == State.STOPPING:
            self.get_logger().error("stopping shutter")
            await self.shutter_stop.call_async(Empty.Request())
            landReq = Land.Request()
            landReq.height = 0.5
            landReq.duration = test
            self.get_logger().error('landing')
            self.state = State.STOPPED
            self.prev_state = State.PAINTING
            await self.cf_land.call_async(landReq)
        



def main(args=None):
    rclpy.init(args=args)
    flight = Flight()
    rclpy.spin(flight)
    rclpy.shutdown()