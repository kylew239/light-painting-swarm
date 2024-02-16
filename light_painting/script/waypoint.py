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


def order(tups_list: List[Tuple]) -> List[Tuple]:
    sortedx = []
    sortedy = []
    curr_point = tups_list.pop()
    sortedx.append(curr_point[0])
    sortedy.append(curr_point[1])

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
        sortedx.append(temp[0])
        sortedy.append(temp[1])
        tups_list.remove(temp)

    return sortedx, sortedy


arrx, arry = order(idx1_list)


scalex = 100
xcaley = 100

# Plotting
fig, ax = plt.subplots()
ax.set_xlim(0, 800)
ax.set_ylim(0, 800)
line, = ax.plot([], [], 'bo', markersize=0.1)

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