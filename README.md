# ros_realtime_plotter
A demo of real-time data collection and plotting for the Robot Operating System in Python

## Installation

+ Install [ROS](http://wiki.ros.org/ROS/Installation) and [gazebo](http://gazebosim.org/tutorials)

+ Create a workspace

```
mkdir -p raz_ws/src
cd raz_ws/src
catkin_init_workspace
```

+ Clone this repo and and the razbot_tutorials repo into the workspace

```
git clone https://github.com/clearpathrobotics/razbot_tutorials.git
git clone https://github.com/CatherineH/ros_realtime_plotter.git
```

+ Install the required python libraries

```
cd ros_realtime_plotter
sudo pip install -r requirements.txt
```

+ Build the package

```
cd ../..
catkin_make
```

## Running

+ Set up the environment variables
```
source devel/setup.bash
```
+ Run the roslaunch file
```
roslaunch ros_realtime_plotter experiment.launch
```

## In Action

![ScreenShot](http://img.youtube.com/vi/vOZMjzghYhc/0.jpg)](https://www.youtube.com/watch?v=vOZMjzghYhc)

