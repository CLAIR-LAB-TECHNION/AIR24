from ur5e_robot import UR5eRobot
import numpy as np


box_grasp_config = [-0.7, -0.98, 1.4, -1.7, -1.4, -0.8]
picked_up_state = [- 0.6, -1.2, 1.4, -2.1, -1.4, -0.8]


if  __name__ == '__main__':
    ur5 = UR5eRobot(max_velocity_scale=0.5)
    ur5.simulate_seconds(0.5)

    state = ur5.get_current_config()
    state = np.array(state)

    ur5.move_to_config(box_grasp_config)
    ur5.close_gripper(0.0)
    ur5.simulate_seconds(2)
    ur5.scale_max_velocities(0.02)
    ur5.move_to_config(picked_up_state)
    ur5.simulate_seconds(2)

    # add set config example

