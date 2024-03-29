# A package for controlling Crazyflie drones to light paints
- Author: Kyle Wang
- [Portfolio Post](https://kylew239.github.io/in_progress/crazyflie/)
- Project Duration: 1/03/24 - 3/16/24

This project uses three Crazyflie Drones to create a long-exposure picture. The camera captures the light emitted by the on-board LEDs over the course of the drones trajectory to create images.

This project utilizes mutliple ROS nodes interfacing with the various hardware and the Crazyflie server ([Crazyswarm2 Package](https://imrclab.github.io/crazyswarm2/)). I created various nodes based on the difference functionalities needed, such as camera control, trajectory generation, and drone control.

# Quickstart
## Setup
1. Clone the repository along with its dependencies, into your ros workspace under the `src/` directory. This can be done using the `vcs install` command.
    * In your ros workspace, run `vcs import --input https://github.com/kylew239/light-painting-swarm/blob/main/light.repos src`
2. Go into the `camera` and `light_painting` repositories and follow the README instructions to setup and configure the system
3. Build using `colcon build` while in the workspace

## Lighthouse setup
1. Follow the instructions [here](https://www.bitcraze.io/documentation/tutorials/getting-started-with-lighthouse/)
2. After setting up one drone, press `Save System Config` under the `System Management` panel
3. Save the file
4. Connect a new drone, and press `Load System Config` under the `System Management` panel, and select the saved file
5. The new drone will have the same geometry data saved


## For light painting with one drone
1. Run `ros2 launch light_painting light_paint_one.launch.py`
2. In a seperate terminal, run `ros2 service call /generate_waypoint light_painting_interfaces/srv/Generate '{filename: <insert file>}'`
    * Be sure to replace `<insert file>` with the full path to your file
3. After getting the service call response, run `ros2 service call /start std_srvs/srv/Empty`

## For light painting with three drones
1. Verify using `visualize.py` that the drones do not fly directly on top of each other. The turbulence will cause them to crash
2. Run `ros2 launch light_painting light_paint_three.launch.py`
3. In a seperate terminal, run `ros2 service call /generate_waypoint light_painting_interfaces/srv/Generate '{filename: <insert file>}'`
    * Be sure to replace `<insert file>` with the full path to your file
4. After getting the service call response, run `ros2 service call /start std_srvs/srv/Empty`

# ROS Packages
- `camera`: A package for controlling the shutter on the DSLR Camera
- `light_painting`: A package for controlling the drone and it's onboard LEDs
- `light_painting_interfaces`: A package for custom interfaces used

# Hardware
* 3x Crazyflie 2.1 running 2023.11 firmware
* 1x Crazyradio 2.0 running 2023.11 firmware
* 2x Lighthouse V2 Base Station
* 1x DSLR Camera 