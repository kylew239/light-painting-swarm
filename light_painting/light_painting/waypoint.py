"""
Generates waypoints from an image input

Parameters:
    - drones (List[String]): A list of drones names
    - left_bound (double): Left of the bounding-box for the waypoint generation
    - right_bound (double): Right of the bounding-box for the waypoint generation
    - bot_bound (double): Bottom of the bounding-box for the waypoint generation
    - top_bound (double): Top of the bounding-box for the waypoint generation
    - resolution (double): Resolution for the waypoint generation
    - y_offset (double): Offset in the y-direction

Services:
    - generate (light_painting_interfaces/Generate): service to input a file for waypoint generation
"""
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from geometry_msgs.msg import Point
from rcl_interfaces.msg import ParameterDescriptor
from light_painting_interfaces.srv import Waypoints, Generate

import cv2
import numpy as np
from typing import List, Tuple
import math


class Waypoint(Node):
    def __init__(self):
        super().__init__("waypoint")
        self.declare_parameter("drones", ["cf231"], ParameterDescriptor(
            description="Names of the drones"))
        self.declare_parameter("left_bound", -0.15, ParameterDescriptor(
            description="Left of the bounding-box for the waypoints"))
        self.declare_parameter("right_bound", 2.25, ParameterDescriptor(
            description="Right of the bounding-box for the waypoints"))
        self.declare_parameter("bot_bound", 0.30, ParameterDescriptor(
            description="Bottom of the bounding-box for the waypoints"))
        self.declare_parameter("top_bound", 1.7, ParameterDescriptor(
            description="Top of the bounding-box for the waypoints"))
        self.declare_parameter("resolution", 0.015, ParameterDescriptor(
            description="Resolution for the waypoint generation"))
        self.declare_parameter("y_offset", 0.1, ParameterDescriptor(
            description="Y-axis offset for flying the waypoints"))

        self.drones = self.get_parameter(
            "drones").get_parameter_value().string_array_value
        self.left = self.get_parameter(
            "left_bound").get_parameter_value().double_value
        self.right = self.get_parameter(
            "right_bound").get_parameter_value().double_value
        self.top = self.get_parameter(
            "top_bound").get_parameter_value().double_value
        self.bot = self.get_parameter(
            "bot_bound").get_parameter_value().double_value
        self.res = self.get_parameter(
            "resolution").get_parameter_value().double_value
        self.y_offset = self.get_parameter(
            "y_offset").get_parameter_value().double_value

        # Use a singular callback group to ensure services don't hang
        self.cb_group = MutuallyExclusiveCallbackGroup()

        # Service Server
        self.toggle = self.create_service(Generate,
                                          "generate_waypoint",
                                          self.waypoint_cb)

        # Service clients for uploading
        # This will accept an arbitrary number of drones
        self.uploads = []
        for drone in self.drones:
            self.uploads.append(self.create_client(Waypoints,
                                                   drone + "/upload",
                                                   callback_group=self.cb_group))

    async def waypoint_cb(self, request, response):
        """Generates the waypoints to fly through."""
        filename = request.filename

        # Generate waypoints
        points_detected = edge_detect(filename)  # working
        (arrx, arry) = generate_waypoints(points_detected,
                                          self.left,
                                          self.bot,
                                          self.right,
                                          self.top,
                                          self.res)

        points_list = []
        yaw_list = []
        for i in range(len(arrx)):
            points_list.append(Point(x=arrx[i],
                                     y=self.y_offset,
                                     z=arry[i]))
            yaw_list.append(0)

        self.get_logger().info(f"Finished generating {len(points_list)} waypoints")

        # Generate waypoint upload request and call each one
        interval = len(points_list)/len(self.drones)
        waypoint_req = Waypoints.Request()
        for i in range(len(self.drones)):
            waypoint_req.point = points_list[int(i*interval):int((i+1)*interval)]
            waypoint_req.yaw = yaw_list[int(i*interval):int((i+1)*interval)]
            self.uploads[i].call_async(waypoint_req)

        return response


def main(args=None):
    rclpy.init(args=args)
    waypoint = Waypoint()
    rclpy.spin(waypoint)
    rclpy.shutdown()


def edge_detect(img: str,
                t1: int = 400,
                t2: int = 500,
                show: bool = False) -> List[Tuple]:
    """
    Finds the pixels that represents the edges in an image,\
    using Canny Edge Detection

    Args:
        img (str): Filename of the image
        t1 (int, optional): First threshold value for edge\
            detection. Defaults to 400.
        t2 (int, optional): Second threshold value for edge\
            detection. Defaults to 500.

    Returns
    -------
        A list of tuples: A list of pixels
    """
    img = cv2.imread(img)

    if show:
        cv2.imshow("Original", img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    # get grayscale and blur for better edge detection
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    img = cv2.GaussianBlur(img, (3, 3), 0)

    if show:
        cv2.imshow("Grayscale and blur", img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    # Canny edge detection
    img = cv2.Canny(image=img, threshold1=t1, threshold2=t2)

    if show:
        cv2.imshow("Canny Edge Detection", img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    # Get list of pixels represnting the edges
    idx = np.nonzero(img)
    return list(zip(idx[0], idx[1]))


def adjacent(p1: Tuple,
             p2: Tuple) -> bool:
    """
    Checks if two points are adjacent to each other

    Args:
        p1 (Tuple): first point
        p2 (Tuple): 2nd point

    Returns
    -------
        bool: If the points are close
    """
    x1, y1 = p1
    x2, y2 = p2
    # if x are the same, and if y differ by one
    if x1 == x2:
        if abs(y1-y2) == 1:
            return True

    # if y are the same, and if x differ by one
    elif y1 == y2:
        if abs(x1-x2) == 1:
            return True

    return False


def dist(p1: Tuple,
         p2: Tuple) -> float:
    """
    Calculates distance between two points.

    Args:
        p1 (Tuple): point 1
        p2 (Tuple): Point 2

    Returns
    -------
        float: Distance
    """
    x1, y1 = p1
    x2, y2 = p2
    dx = x1-x2
    dy = y1-y2

    return math.sqrt(dx*dx + dy*dy)


def generate_waypoints(idx_list: List[Tuple],
                       xleft: float,
                       ybot: float,
                       xright: float,
                       ytop: float,
                       resolution: float,
                       uniform_scale: bool = True) -> (List[float], List[float]):
    """
    Generate a list of waypoints.

    Args:
        idx_list (List(Tuple)): A list of points generated by edge detection and np.nonzero.\
                                Represents an image
        xleft (float): Left x-coordinate of the bounding box for the waypoints
        ybot (float): Bottom y-coordinate of the bounding box for the waypoints
        xright (float): Right x-coordinate of the bounding box for the waypoints
        ytop (float): Top y-coordinate of the bounding box for the waypoints
        uniform_scale (bool, optional): Determines if uniform scaling is used to preserve the\
                                        original shape of the image. Defaults to True

    Returns
    -------
        Two lists: A list of x-values and a list of y-values representing the waypoints
    """
    # Rotate and flip the image
    points = [(x, -y) for y, x in idx_list]

    sortedx = []
    sortedy = []
    curr_point = points.pop()
    sortedx.append(curr_point[0])
    sortedy.append(curr_point[1])
    xmin, xmax = curr_point[0], curr_point[0]
    ymin, ymax = curr_point[1], curr_point[1]

    # while list isn't empty, keep searching
    while len(points) > 0:
        temp = None
        close = None
        small_dist = math.inf

        # go through each point and find the next one to connect to
        for point in points:
            # update closest neighbor
            if (dist(point, curr_point) < small_dist):
                close = point
                small_dist = dist(point, curr_point)

            # Check if the next point is one pixel away
            # store it, and exit the loop
            if (adjacent(curr_point, point)):
                temp = point
                break  # exits the upper for loop

        # if there is no next point
        if temp == None:
            temp = close

        # update point, add to array, and remove from list
        curr_point = temp
        sortedx.append(curr_point[0])
        sortedy.append(curr_point[1])
        points.remove(curr_point)

        # update mins and maxs
        xmin = min(xmin, curr_point[0])
        xmax = max(xmax, curr_point[0])
        ymin = min(ymin, curr_point[1])
        ymax = max(ymax, curr_point[1])

    # Calculate scaling values
    xscale = (xright-xleft)/(xmax-xmin)
    yscale = (ytop-ybot)/(ymax-ymin)

    if uniform_scale:
        # Scale to the min value
        scale = min(xscale, yscale)

        # Calculate centers
        xcenter = (xleft+xright)/2
        ycenter = (ytop+ybot)/2

        # Calculate center of original image
        xcenter_img = (xmin+xmax)/2
        ycenter_img = (ymin+ymax)/2

        # Scale and shift using centers to make sure image is centered
        sortedx = [xcenter + (point-xcenter_img)*scale for point in sortedx]
        sortedy = [ycenter + (point-ycenter_img)*scale for point in sortedy]

    else:
        # Non-uniform scaling would force the imagee to fit the whole
        # area, so centering is not needed
        sortedx = [xleft+(point-xmin)*xscale for point in sortedx]
        sortedy = [ybot+(point-ymin)*yscale for point in sortedy]

    # remove unnecessary points based on resolution
    currx = math.inf
    curry = math.inf
    x = []
    y = []

    for i in range(len(sortedx)-1):
        if (dist((currx, curry), (sortedx[i], sortedy[i])) > resolution):
            currx = sortedx[i]
            curry = sortedy[i]
            x.append(currx)
            y.append(curry)

    return (x, y)
