from ur5e_robot import UR5eRobot
import numpy as np


if  __name__ == '__main__':
    robot_x = -0.165
    box_x = 0.421
    robot_y = 0.51
    box_y = 0.267

    # position of the box, relative to the robot frame:
    target_x = robot_x - box_x
    target_y = robot_y - box_y
    target_z = 0.2  # a little be above the box
    above_box_position = [target_x, target_y, target_z]

    orientation = [np.pi, 0, 0] # gripper facing downwards

    ur5 = UR5eRobot()
    ur5.simulate_seconds(0.1)

    # Rotate the base of the robot 180 degrees from its starting position. you can remove these lines,
    # see what happens and think about it.
    start_config = ur5.get_current_config()
    start_config[0] += np.pi
    ur5.move_to_config(start_config)

    # move above the box:
    ur5.move_to_pose_with_ik(above_box_position, orientation)

    # grasp the box:
    box_grasp_position = above_box_position.copy()
    box_grasp_position[2] = 0.1
    ur5.move_to_pose_with_ik(box_grasp_position, orientation, max_time=2)
    ur5.close_gripper_and_wait(0.0, 3)

    # pick up the box:
    ur5.move_to_pose_with_ik(above_box_position, orientation, max_time=2)

    ur5.scale_max_velocities(0.1) # what happens if you remove this line?
    # move to the drop-off position and drop the box:
    ur5.move_to_pose_with_ik([-0.3, 0.1, 0.2], orientation)
    ur5.open_gripper_and_wait(3)

