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

# Nodes

