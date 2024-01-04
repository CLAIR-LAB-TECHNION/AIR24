from ur5e_robot import UR5eRobot
import numpy as np


box_grasp_config = [-0.7, -0.98, 1.4, -1.7, -1.4, -0.8]
picked_up_state = [- 0.6, -1.2, 1.4, -2.1, -1.4, -0.8]


if  __name__ == '__main__':
    # create a robot instance, there are default arguments,
    # read the __init__ method documentation for more information
    ur5 = UR5eRobot()

    # run the simulation for half a second:
    ur5.sicontrol
    conds(0.5)
    # you can run it for one step as well, a step is 32 milliseconds by default:
    ur5.robot_step()

    # get the current configuration of the robot, it's a 6d vector of the joint states
    state = ur5.get_current_config()
    print(state)  # will print a vector of 6 floats, corresponding to the joint angles in radians

    # move the robot by move_to_config, this method will run the simulation until the robot reaches the target
    # config or fails. It will return True if the robot reached the target config, False otherwise.
    res = ur5.move_to_config(box_grasp_config)
    print(res)  # will print True if the robot reached the target config, False otherwise.

    # the gripper should be now above the box, close it and run the simulation for 2 seconds to let it close:
    ur5.close_gripper(0.0)
    ur5.simulate_seconds(2)
    # we could have called robot.close_gripper_and_wait(0.0, 2) instead, and it would have done the same thing.

    # we are going to pick up the box now, but if the robot will move too fast it will lose grasp and the box
    # will fall, so we need to slow down the robot, we can do that by scaling the max velocities of the robot:
    ur5.scale_max_velocities(0.02)
    # we will now move the robot to a picked up state, but instead of using move_to_config we will run the
    # simulation manually:
    ur5.set_target_config(picked_up_state)
    ur5.simulate_seconds(5)

    # let's open the gripper and drop the box:
    ur5.open_gripper_and_wait(2)

    # run the simulationfor a few more seconds, then the controller will terminate and simulation will stop:
    ur5.simulate_seconds(10)

