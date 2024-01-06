Build package:
> colcon build

Launch example:
> source install/local_setup.bash
> ros2 launch clair_package ur5_launch.py

Then, when simulation is up, send a message to the topic to move the robot:
> ros2 topic pub --once /target_joint_states sensor_msgs/msg/JointState '{position: [-0.7, -0.98, 1.4, -1.7, -1.4, -0.8]}'

The robot should move