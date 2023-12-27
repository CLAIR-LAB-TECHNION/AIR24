import numpy as np
from controller import Robot, Supervisor

UR5e_motor_indices = [0, 2, 4, 6, 8, 10]
UR5e_sensor_indices = [1, 3, 5, 7, 9, 11]

joints_init_state = [-3.7, 1.2, 1.2, 1.2, 1.2, 1.2]


class Gripper:
    def __init__(self, robot):
        self._robot = robot

        self.finger_l = robot.getDevice('ROBOTIQ 2F-140 Gripper::left finger joint')
        self.finger_r = robot.getDevice('ROBOTIQ 2F-140 Gripper::right finger joint')

        self.finger_l.setAvailableTorque(10)
        self.finger_r.setAvailableTorque(10)
        self.finger_l.setVelocity(0.2)
        self.finger_r.setVelocity(0.2)

        self.l_limits = (self.finger_l.getMinPosition(), self.finger_l.getMaxPosition())
        self.r_limits = (self.finger_r.getMinPosition(), self.finger_r.getMaxPosition())

    def close(self, gap=0.0):
        '''
        :param gap: The gap to leave, between 0 and 1 where 0 is fully closed
                and 1 is open
        '''
        position_left = self.l_limits[1] + gap * (self.l_limits[0] - self.l_limits[1])
        position_right = self.r_limits[1] + gap * (self.r_limits[0] - self.r_limits[1])
        self.finger_l.setPosition(position_left)
        self.finger_r.setPosition(position_right)

    def open(self):
        self.finger_l.setPosition(self.l_limits[0])
        self.finger_r.setPosition(self.r_limits[0])


class UR5eRobot:
    """ a class to abstract control over the UR5e robot in Webots"""

    def __init__(self, interval=32, max_velocity_scale=0.5, robot_name=None, initial_state=None):
        """

        :param interval: The robot control interval (time between steps) in milliseconds.
        :param robot_name: Robot name in the Webots world. If None, it will be set automatically
            only if there is only one robot.
        :param initial_state: Initial position of the robot. If None, it will be set to a default one
        """
        self._robot = Robot()
        self.interval = interval

        self.joint_motors = [self._robot.getDeviceByIndex(i) for i in UR5e_motor_indices]
        self.joint_sensors = [self._robot.getDeviceByIndex(i) for i in UR5e_sensor_indices]
        self.joint_names = [motor.getName() for motor in self.joint_motors]

        for sensor in self.joint_sensors:
            sensor.enable(self.interval)

        # save original joints max velocity:
        self.original_max_velocities = [motor.getMaxVelocity() for motor in self.joint_motors]
        self.scale_max_velocities(max_velocity_scale)

        self.gripper = Gripper(self._robot)
        self.gripper.open()
        self.robot_step()

    def robot_step(self):
        '''
        advance one simulation step (for interval length)
        '''
        self._robot.step(self.interval)

    def simulate_seconds(self, seconds):
        '''
        advance in the simulation for a given number of seconds
        '''
        for _ in range(int(seconds * 1000 / self.interval)):
            self.robot_step()

    def set_target_config(self, joint_states):
        for motor, state in zip(self.joint_motors, joint_states):
            motor.setPosition(state)
        self.robot_step()

    def get_current_config(self):
        return [sensor.getValue() for sensor in self.joint_sensors]

    def move_to_config(self, joint_states, max_err=1e-3, max_time=5):
        """
        move to a desired config, this method will run simulation until the robot is close enough
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
        close the gripper to a given gap between 0 and 1 (0 is fully closed, 1 is open)
        :param gap:
        """
        self.gripper.close(gap)
        self.robot_step()

    def open_gripper(self):
        '''
        open the gripper all the way
        '''
        self.gripper.open()
        self.robot_step()

    def scale_max_velocities(self, max_velocity_scale):
        '''
        scale the max velocities of the joints
        :param max_velocity_scale: the scale to apply to the max velocities between 0 and 1
            1 means robot full velocity.
        '''
        for motor, orig_vel in zip(self.joint_motors, self.original_max_velocities):
            motor.setVelocity(max_velocity_scale * orig_vel)
