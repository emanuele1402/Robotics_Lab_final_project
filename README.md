Use Dockerfile in this repository.

Put final_project and and [aruco_ros](https://github.com/pal-robotics/aruco_ros) in ros2_ws/src
```
git clone https://github.com/emanudevito14/final_project.git
```
In the terminal
```
colcon build
source install/setup.bash
```
```
ros2 launch final_project config.launch.py command_interface:="velocity" robot_controller:="kuka_velocity_controller"
```

Open new terminal
```
ros2 launch final_project amr_nav.launch.py

```
Open new terminal 
```
ros2 launch final_project kuka.launch.py
```
