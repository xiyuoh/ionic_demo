# ionic_demo
Ionic demo world and resources

![image](https://github.com/user-attachments/assets/4b6da5bc-d2ce-4b5f-ac97-f6f238200123)

Usage:

```
git clone https://github.com/gazebosim/ionic_demo
cd ionic_demo/ionic_demo/worlds
gz sim -v 4 ionic.sdf
```

In Harmonic:
```
sudo apt install ros-rolling-ros-gz
```

# Running the navigation demo

This demo requires at least [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/index.html).

Create a new colcon workspace, install dependencies and build the packages,

```
mkdir -p ~/ionic_ws/src
cd ~/ionic_w/src
git clone https://github.com/gazebosim/ionic_demo

source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y

cd ~/ionic_ws/
colcon build
```

Launch the demo,

```
source ~/ionic_ws/install/setup.bash
ros2 launch ionic_demo ionic_navigation_demo_launch.py headless:=0
```

On rviz, initialize the position at the origin towards the right of the map.

Give navigation commands as per normal.


# Troubleshooting

* If there are communication/middleware related issues while running the demos, we recommend trying again [using a different RMW implementation](https://docs.ros.org/en/jazzy/How-To-Guides/Working-with-multiple-RMW-implementations.html#specifying-rmw-implementations), for example `rmw_cyclonedds_cpp`.
