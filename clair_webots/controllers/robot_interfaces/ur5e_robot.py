import numpy as np
from controller import Robot, Supervisor
from math import cos, sin
from inverse_kinematics.inverse_kinematics import forward_kinematic_solution, inverse_kinematic_solution,\
    get_DH_matrix_UR5e
from scipy.spatial.transform import Rotation


UR5e_motor_indices = [0, 2, 4, 6, 8, 10]
UR5e_sensor_indices = [1, 3, 5, 7, 9, 11]


class UR5eRobot:
    """
    a class to abstract control over the UR5e robot in Webots, without a tool.
    This class can be extended to robot with a tool such as gripper or camera by
    implementing its functionality in the child class.
    The goal of this class is to provide interface to robot motion.
    """

    def __init__(self, interval=32, max_velocity_scale=0.5, robot_name=None):
        """
        :param interval: The robot control interval (time between steps) in milliseconds.
        :param max_velocity_scale: The scale to apply to the max velocities between 0 and 1, where 1 is the robot
            default velocity, which is quite fast and not that safe.
        :param robot_name: Robot name in the Webots world. If None, it will be set automatically
            only if there is only one robot.
        """
        # TODO: handle robot name
        self._robot = Supervisor()
        self._robot_node = self._robot.getSelf()
        self.robot_position = self._robot_node.getField('translation').getSFVec3f()
        robot_rotation_quat = self._robot_node.getField('rotation').getSFRotation()
        # rotation is in axis direction, rotation angle format. change to euler:
        R = Rotation.from_rotvec(robot_rotation_quat[3] * np.array(robot_rotation_quat[:3]))
        self.robot_rotation = R.as_euler('xyz', degrees=False)

        self.interval = interval

        self._joint_motors = [self._robot.getDeviceByIndex(i) for i in UR5e_motor_indices]
        self._joint_sensors = [self._robot.getDeviceByIndex(i) for i in UR5e_sensor_indices]
        self._joint_names = [motor.getName() for motor in self._joint_motors]

        for sensor in self._joint_sensors:
            sensor.enable(self.interval)

        # save original joints max velocity:
        self.original_max_velocities = [motor.getMaxVelocity() for motor in self._joint_motors]
        self.scale_max_velocities(max_velocity_scale)

        self.tool_length = 0.135  # default, can be set by child classes

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

        iks = inverse_kinematic_solution(get_DH_matrix_UR5e(self.tool_length), transform)
        return iks[:, 0]

    def set_target_pose_with_ik(self, position, orientation):
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

    def get_ee_pose_robot(self,):
        '''
        Get the end effector pose in robot frame
        :return: end effector position, end effector rotation euler
        '''
        transform = forward_kinematic_solution(get_DH_matrix_UR5e(self.tool_length), self.get_current_config())
        position = np.array(transform[:3, 3]).reshape(3)
        rotation = transform[:3, :3]

        R = Rotation.from_matrix(rotation)
        rotation_euler = R.as_euler('xyz', degrees=False)

        return position, rotation_euler

    def get_ee_pose_world(self):
        '''
        Get the end effector pose in world frame
        :return: end effector position, end effector rotation euler
        '''
        position_r_frame, rotation_euler_r_frame = self.get_ee_pose_robot()
        robot_position = np.array(self.robot_position)
        robot_rotation = np.array(self.robot_rotation)

        robot_rotation_matrix = Rotation.from_euler('xyz', robot_rotation, degrees=False).as_matrix()
        rotation_in_r_frame_matrix = Rotation.from_euler('xyz', rotation_euler_r_frame, degrees=False).as_matrix()
        world_rotation_matrix = rotation_in_r_frame_matrix @ robot_rotation_matrix
        world_rotation_euler = Rotation.from_matrix(world_rotation_matrix).as_euler('xyz', degrees=False)

        world_position = robot_position + robot_rotation_matrix @ np.array(position_r_frame).T

        return world_position, world_rotation_euler