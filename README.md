# ros2_checkersgame_ur5e

## How to run
1. In 1st terminal launch UR5-e driver: 
```
ros2 launch urk_ur_launch ur5e_cocohrip.launch.py
```
2. In 2nd terminal launch UR5-e moveIt with RViz simulation and in RViz add MarkerArrray: 
```
ros2 launch urk_ur_launch ur_cocohrip_moveit.launch.py
```
3. In 3rd terminal launch gripper Robotiq Hand-E:
```
ros2 launch robotiq_hande_ros2_driver gripper_bringup.launch.py
```
4. In 4th terminal launch Leap Motion sensor:
```
ros2 run leap_gesture_interface leap_example_node
```
5. In 5th terminal launch main program for control robot:
```
ros2 launch robot_control robot_control_launch.py 
```
5. In 5th terminal launch main program for camera Ximea and game logic:
```
ros2 launch checkers_game checkers_game_launch.launch.py
```

### How to run following task from terminal
1. Resume robot movement:
```
ros2 service call /resume_movement checkers_msgs/srv/ResumeMovement 
```
