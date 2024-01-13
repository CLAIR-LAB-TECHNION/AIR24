import numpy as np
from controller import Robot, Supervisor
from math import pi, cos, sin, atan2, acos, sqrt, asin
from inverse_kinematics.inverse_kinematics import inverse_kinematic_solution, DH_matrix_UR5e


UR5e_motor_indices = [0, 2, 4, 6, 8, 10]
UR5e_sensor_indices = [1, 3, 5, 7, 9, 11]

# joints_init_state = [-3.7, 1.2, 1.2, 1.2, 1.2, 1.2]


class Gripper:
    """ a class to abstract control over the _gripper in Webots ROBITIQ 2 finger _gripper """
    def __init__(self, robot):
        self._robot = robot

        self._finger_l = robot.getDevice('ROBOTIQ 2F-140 Gripper::left finger joint')
        self._finger_r = robot.getDevice('ROBOTIQ 2F-140 Gripper::right finger joint')

        self._finger_l.setAvailableTorque(10)
        self._finger_r.setAvailableTorque(10)
        self._finger_l.setVelocity(0.2)
        self._finger_r.setVelocity(0.2)

        self._l_limits = (self._finger_l.getMinPosition(), self._finger_l.getMaxPosition())
        self._r_limits = (self._finger_r.getMinPosition(), self._finger_r.getMaxPosition())

    def close(self, gap=0.0):
        '''
        :param gap: The gap to leave, between 0 and 1 where 0 is fully closed
                and 1 is open
        '''
        position_left = self._l_limits[1] + gap * (self._l_limits[0] - self._l_limits[1])
        position_right = self._r_limits[1] + gap * (self._r_limits[0] - self._r_limits[1])
        self._finger_l.setPosition(position_left)
        self._finger_r.setPosition(position_right)

    def open(self):
        self._finger_l.setPosition(self._l_limits[0])
        self._finger_r.setPosition(self._r_limits[0])


class UR5eRobot:
    """ a class to abstract control over the UR5e robot in Webots"""

    def __init__(self, interval=32, max_velocity_scale=0.5, robot_name=None):
        """
        :param interval: The robot control interval (time between steps) in milliseconds.
        :param robot_name: Robot name in the Webots world. If None, it will be set automatically
            only if there is only one robot.
        :param max_velocity_scale: The scale to apply to the max velocities between 0 and 1, where 1 is the robot
            default velocity, which is quite fast and not that safe.
        :param initial_state: Initial position of the robot. If None, it will be set to a default one
        """
        self._robot = Robot()
        self.interval = interval

        self._joint_motors = [self._robot.getDeviceByIndex(i) for i in UR5e_motor_indices]
        self._joint_sensors = [self._robot.getDeviceByIndex(i) for i in UR5e_sensor_indices]
        self._joint_names = [motor.getName() for motor in self._joint_motors]

        for sensor in self._joint_sensors:
            sensor.enable(self.interval)

        # save original joints max velocity:
        self.original_max_velocities = [motor.getMaxVelocity() for motor in self._joint_motors]
        self.scale_max_velocities(max_velocity_scale)

        self._gripper = Gripper(self._robot)
        self._gripper.open()
        self.robot_step()

    def robot_step(self):
        """
        advance one simulation step (for self.interval length)
        """
        self._robot.step(self.interval)

    def simulate_seconds(self, seconds):
        """
        advance in the simulation for a given number of seconds
        :param seconds: number of seconds to advance
        """
        for _ in range(int(seconds * 1000 / self.interval)):
            self.robot_step()

    def set_target_config(self, joint_states):
        """
        set the target config of the robot, in the next simulation steps the robot will move to this config
        :param joint_states: 6d vector of the desired joint states
        :return:
        """
        for motor, state in zip(self._joint_motors, joint_states):
            motor.setPosition(state)
        self.robot_step()

    def get_current_config(self):
        """
        get the current config of the robot
        :return: 6d vector of the current joint states
        """
        return [sensor.getValue() for sensor in self._joint_sensors]

    def move_to_config(self, joint_states, max_err=1e-3, max_time=5):
        """
        move to a desired config, this method will run simulation until the robot is close enough.
        The robot might not be able to reach the target if there is an obstacle in the way.
        :param joint_states: the desired joint states
        :param max_err: maximum norm of error to define target arrival
        :param max_time: maximum time to try to reach the target
        :return: True if target is reached, false otherwise
        """
        joint_states = np.array(joint_states)
        self.set_target_config(joint_states)
        for _ in range(int(max_time * 1000 / self.interval)):
            self.robot_step()
            current_config = np.array(self.get_current_config())
            if np.linalg.norm(current_config - joint_states) < max_err:
                return True
        return False

    def close_gripper(self, gap=0.0):
        """
        close the _gripper to a given gap between 0 and 1 (0 is fully closed, 1 is open)
        :param gap:
        """
        self._gripper.close(gap)
        self.robot_step()

    def open_gripper(self):
        """
        open the _gripper all the way
        """
        self._gripper.open()
        self.robot_step()

    def close_gripper_and_wait(self, gap=0.0, time=2):
        """
        close the _gripper to a given gap between 0 and 1 (0 is fully closed, 1 is open) and run the simulation
        to let it close
        :param gap:
        :param time: time to run simulation after _gripper close command
        :return:
        """
        self.close_gripper(gap)
        self.simulate_seconds(time)

    def open_gripper_and_wait(self, time=2):
        '''
        open the _gripper all the way and run the simulation to let it open
        :param time: time to run simulation after _gripper open command
        :return:
        '''
        self.open_gripper()
        self.simulate_seconds(time)

    def scale_max_velocities(self, max_velocity_scale):
        '''
        scale the max velocities of the joints
        :param max_velocity_scale: the scale to apply to the max velocities between 0 and 1
            1 means robot full velocity.
        '''
        for motor, orig_vel in zip(self._joint_motors, self.original_max_velocities):
            motor.setVelocity(max_velocity_scale * orig_vel)


    def get_ik_solution(self, position, orientation):
        '''
        Get the inverse kinematics solution for the desired position and orientation of the end effector.
        The position and angle are relative to the robot's frame of reference and not to the world!
        :param position: 3d vector of the desired position
        :param orientation: 3d vector of the desired orientation in euler angles
        '''
        alpha, beta, gamma = orientation
        tx, ty, tz = position

        alpha, beta, gamma = orientation
        tx, ty, tz = position
        transform = np.matrix([[cos(beta) * cos(gamma), sin(alpha) * sin(beta) * cos(gamma) - cos(alpha) * sin(gamma),
                                cos(alpha) * sin(beta) * cos(gamma) + sin(alpha) * sin(gamma), tx],
                               [cos(beta) * sin(gamma), sin(alpha) * sin(beta) * sin(gamma) + cos(alpha) * cos(gamma),
                                cos(alpha) * sin(beta) * sin(gamma) - sin(alpha) * cos(gamma), ty],
                               [-sin(beta), sin(alpha) * cos(beta), cos(alpha) * cos(beta), tz],
                               [0, 0, 0, 1]])

        iks = inverse_kinematic_solution(DH_matrix_UR5e, transform)
        return iks[:, 0]


    def set_tartget_pose_with_ik(self, position, orientation):
        '''
        Set the target pose of the robot using inverse kinematics to get the desired joint states from the desired
        position and orientation of the end effector. The position and angle are relative to the robot's frame of
        reference and not to the world!
        :param position: 3d vector of the desired position
        :param orientation: 3d vector of the desired orientation in euler angles
        '''

        iks = self.get_ik_solution(position, orientation)
        self.set_target_config(iks)

    def move_to_pose_with_ik(self, position, orientation, max_err=1e-3, max_time=5):
        '''
        Move the robot to a desired pose using inverse kinematics to get the desired joint states from the desired
        position and orientation of the end effector. The position and angle are relative to the robot's frame of
        reference and not to the world!
        :param position: 3d vector of the desired position
        :param orientation: 3d vector of the desired orientation in euler angles
        :param max_err: maximum norm of error to define target arrival
        :param max_time: maximum time to try to reach the target
        :return: True if target is reached, false otherwise
        '''
        iks = self.get_ik_solution(position, orientation)
        return self.move_to_config(iks, max_err, max_time)