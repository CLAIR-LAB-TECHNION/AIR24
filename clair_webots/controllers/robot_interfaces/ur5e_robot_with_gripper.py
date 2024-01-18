from robot_interfaces.ur5e_robot import UR5eRobot


class Gripper:
    """ a class to abstract control over the _gripper in Webots ROBITIQ 2 finger _gripper """
    def __init__(self, robot):
        self._robot = robot

        # make sure that robot has gripper:
        device_names = [robot.getDeviceByIndex(i).getName() for i in range(robot.getNumberOfDevices())]
        assert 'ROBOTIQ 2F-140 Gripper::left finger joint' in device_names, "robot doesn't have a supported gripper"

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


class UR5eRobotWithGripper(UR5eRobot):
    """Interface to ar UR5e robot with a gripper"""

    def __init__(self, interval=32, max_velocity_scale=0.5, robot_name=None):
        super().__init__(interval, max_velocity_scale, robot_name)

        self._gripper = Gripper(self._robot)
        self._gripper.open()
        self.robot_step()

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

