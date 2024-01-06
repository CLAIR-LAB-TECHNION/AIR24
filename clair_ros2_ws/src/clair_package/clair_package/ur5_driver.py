import rclpy
from sensor_msgs.msg import JointState


UR5e_motor_indices = [0, 2, 4, 6, 8, 10]
UR5e_sensor_indices = [1, 3, 5, 7, 9, 11]

class UR5Driver:
    def init(self, webots_node, properties):
        # print('UR5Driver.__init__')
        self._robot = webots_node.robot

        self._joint_motors = [self._robot.getDeviceByIndex(i) for i in UR5e_motor_indices]
        self._joint_names = [motor.getName() for motor in self._joint_motors]
        self._joint_sensors = [self._robot.getDeviceByIndex(i) for i in UR5e_sensor_indices]
        for sensor in self._joint_sensors:
            sensor.enable(32)

        self._target_position = JointState()
        self._target_position.name = self._joint_names

        rclpy.init()

        self._node = rclpy.create_node('ur5_driver')
        self._node.create_subscription(JointState, 'target_joint_states', self._joint_state_callback, 1)

        self._joint_state_publisher = self._node.create_publisher(JointState, 'joint_states', 10)

        self._logger = self._node.get_logger()

    def _joint_state_callback(self, target_position):
        self._target_position = target_position

    def step(self):
        rclpy.spin_once(self._node, timeout_sec=0)

        if not self._target_position.position:
            self._logger.debug(f'no position is specified for joints, skipping step')
            return

        # if there are names, target position is specified by names:
        if self._target_position.name:
            for joint_name, pos in zip(self._target_position.name, self._target_position.position):
                assert joint_name in self._joint_names, f'joint name {joint_name} not in {self._joint_names}'
                self._robot.getDevice(joint_name).setPosition(pos)
        else:
            # otherwise, target position is specified by indices:
            for i, pos in enumerate(self._target_position.position):
                self._joint_motors[i].setPosition(pos)

        self._publish_joint_states()

    def _publish_joint_states(self, ):
        joint_state = JointState()
        joint_state.name = self._joint_names
        joint_state.position = self._get_webots_joint_positions()
        self._joint_state_publisher.publish(joint_state)

    def _get_webots_joint_positions(self):
        return [s.getValue() for s in self._joint_sensors]
