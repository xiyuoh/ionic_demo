# ionic_demo
Ionic demo world and resources

![image](https://github.com/user-attachments/assets/4b6da5bc-d2ce-4b5f-ac97-f6f238200123)

Usage:

```
cd Ionic
gz sim -v 4 ionic.sdf
```


In Harmonic:
```
sudo apt install ros-rolling-ros-gz
```

# navigation demo

```
# Launches the simulation with a spawned turtlebot4
ros2 launch ionic_demo ionic_navigation_demo_launch.py headless:=0
```

On rviz, initialize the position at the origin towards the right of the map.

Give navigation commands as per normal.
