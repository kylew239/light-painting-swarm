# The ROS package responsible for controlling the drones
This package contains the LED controller, a flight controller wrapper, and a waypoint generator. This package contains most of the nodes and launchfiles required.

# Quickstart
1. Go to `config/config.yaml` and modify the drone URI to use the correct radio and address. Refer to the official crazyflie [start guide](https://www.bitcraze.io/start/)
2. Use the `script/waypoint.py` python script to verify your image's result is what you expect
    * Modify the file name on line 221.
    * On lines 222-225, change your expected bounding box for the image
    * Run the script. It should open images displaying the intermediate processing steps, along with an animation of the final path.
3. Go to `launch/light_paint.launch.py` and modify the bounding box to the size you want

# Launch options
```
Arguments (pass arguments as '<name>:=<value>'):

    'led_control':
        Determines the LED controller mode (radius | line)
        (default: 'radius')

    'threshold':
        The threshold for the LED controller to use
        (default: '0.03')
```

# Setting up n-number of drones
1. Make a new launch file
2. Copy over the code from `light_paint_three.launch.py`
3. Copy and paste the `flight` and `led` nodes such that there is one for each drone
    * Change the `drone` parameter to match the name of the drone
    * Change the `y_offset` parameter if the drones need to be offset in the y direction
    * Change the `color` parameter as desired. The onboard LEDs have red, green, and blue options
4. In the `config/config.yaml` file, add any additional drones, following the template
    * Each drone should be at least 2MB apart
    * Each drone should have a different address and name (cf1, cf2, etc)

# Nodes
* `waypoint`: Responsible for generating waypoints from an image using OpenCV and a nearest-neighbors algorithm
* `led`: Responsible for controlling the onboard status LED to turn on and off depending on the control method and proximity
* `flight`: Responsible for controlling the flight of the drone. Directly interfaces with `crazyflie_server`

# Scripts
* `canny.py`: Visualizes the output of Canny edge detection
* `visualize.py`: Takes in a video and traces the trajectory of the drone
* `waypoint.py`: Visualizes the waypoint generation process
