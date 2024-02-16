import cv2
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from typing import List, Tuple
import math


# Read the original image
img = cv2.imread('tree.jpg')
img2 = cv2.imread('cube.png')

# Convert to graycsale
img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
img2_gray = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)

# Blur the image for better edge detection
img_blur = cv2.GaussianBlur(img_gray, (3, 3), 0)
img2_blur = cv2.GaussianBlur(img2_gray, (3, 3), 0)

# Edge Detection
t1 = 400
t2 = 500
edges = cv2.Canny(image=img_blur, threshold1=t1,
                  threshold2=t2)  # Canny Edge Detection
edges2 = cv2.Canny(image=img2_blur, threshold1=t1,
                   threshold2=t2)  # Canny Edge Detection

# Display Canny Edge Detection Image

idx1 = np.nonzero(edges)
idx1_list = list(zip(idx1[0], idx1[1]))

idx2 = np.nonzero(edges2)
idx2_list = list(zip(idx2[0], idx2[1]))

# new_list = idx1_list[::5]
# for item in new_list:
#     x = float(item[1]) / 100.0
#     y = float(item[0]) / -100.0
#     print("(", x, ",", y, ")")


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


def generate_waypoints(tups_list: List[Tuple],
                       xsize: float = None,
                       ysize: float = None,
                       xmargin: float = 0.0,
                       ymargin: float = 0.0) -> List[Tuple]:
    sortedx = []
    sortedy = []
    curr_point = tups_list.pop()
    sortedx.append(curr_point[0])
    sortedy.append(curr_point[1])
    xmin, xmax = curr_point[0], curr_point[0]
    ymin, ymax = curr_point[1], curr_point[1]

    # while list isn't empty, keep searching
    while len(tups_list) > 0:
        # go through each point and find the next one to connect to
        temp = None
        close = None
        small_dist = math.inf

        for point in tups_list:

            # update closest neighbor
            if (dist(point, curr_point) < small_dist):
                close = point
                small_dist = dist(point, curr_point)

            # Check if the next point is one pixel away
            # store it, and exit the loop
            if (adjacent(curr_point, point)):
                temp = point

                # exits the upper for loop
                break

        # if there is no next point
        if temp == None:
            temp = close

        # update point, add to array, and remove from list
        curr_point = temp
        sortedx.append(curr_point[0])
        sortedy.append(curr_point[1])
        tups_list.remove(curr_point)

        # update mins and maxs
        xmin = min(xmin, curr_point[0])
        xmax = max(xmax, curr_point[0])
        ymin = min(ymin, curr_point[1])
        ymax = max(ymax, curr_point[1])


    # Scale the image
    if xsize is not None or ysize is not None:
        # If either are not set, scale uniformly
        if xsize is None:
            xsize = ysize
        if ysize is None:
            ysize = xsize

        # offset values from axis, calculated using margin percentage
        xoffset = xmargin*xsize
        yoffset = ymargin*ysize

        # Scale to divide by, to fit everything within the margins
        xscale = (xmax-xmin)/(xsize * (1-2*xmargin))
        yscale = (ymax-ymin)/(ysize * (1-2*ymargin))

        # Take each value, shift it over to the axis, and scale down
        # Then add in the calculated offset values
        sortedx = [(point - xmin)/xscale + xoffset for point in sortedx]
        sortedy = [((point - ymin)/yscale + yoffset) for point in sortedy]

    return sortedx, sortedy


xcanvas = 100
ycanvas = 100
arrx, arry = generate_waypoints(idx1_list,
                                xcanvas,
                                ycanvas,
                                0.05,
                                0.05)

# Plotting
fig, ax = plt.subplots()
ax.set_xlim(0, xcanvas)
ax.set_ylim(0, ycanvas)
line, = ax.plot([], [], marker='o', markersize=1, color='black', alpha=1.0, linestyle='None')

# Function to update the plot for each frame of the animation
def update(frame):
    if frame < len(arrx):
        y = arrx[:frame+1]
        x = arry[:frame+1]
        line.set_data(x, y)
        return line,
    else:
        return line,

# Create the animation
ani = FuncAnimation(fig,
                    update,
                    frames=len(arrx)+1,
                    blit=True,
                    interval=1)

plt.show()