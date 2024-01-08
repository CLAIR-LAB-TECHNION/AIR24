import rclpy
from std_msgs.msg import Float32


class GripperDriver2F140:
    def init(self, webots_node, properties):
        self._finger_l_motor = webots_node.robot.getDevice('ROBOTIQ 2F-140 Gripper::left finger joint')
        self._finger_r_motor = webots_node.robot.getDevice('ROBOTIQ 2F-140 Gripper::right finger joint')
        self._finger_r_sensor = webots_node.robot.getDevice('ROBOTIQ 2F-140 Gripper right finger joint sensor')

        self._finger_r_sensor.enable(32)

        self._finger_l_motor.setAvailableTorque(10)
        self._finger_r_motor.setAvailableTorque(10)
        self._finger_l_motor.setVelocity(0.2)
        self._finger_r_motor.setVelocity(0.2)

        self._l_limits = (self._finger_l_motor.getMinPosition(), self._finger_l_motor.getMaxPosition())
        self._r_limits = (self._finger_r_motor.getMinPosition(), self._finger_r_motor.getMaxPosition())

        self._target_position = 1.0

        self._node = rclpy.create_node('gripper_driver_2f140')
        self._node.create_subscription(Float32, 'target_gripper_position', self._gripper_position_callback, 1)

        self._state_publisher = self._node.create_publisher(Float32, 'gripper_position', 5)

        self._logger = self._node.get_logger()

    def _gripper_position_callback(self, target_position):
        target_position = target_position.data
        if not  0.0 <= target_position <= 1.0:
            self._logger.warn(f'gripper position {target_position} is not in [0, 1], ignoring')
            return
        self._target_position = target_position

    def step(self):
        rclpy.spin_once(self._node, timeout_sec=0)
        position_left = self._l_limits[1] + self._target_position * (self._l_limits[0] - self._l_limits[1])
        position_right = self._r_limits[1] + self._target_position * (self._r_limits[0] - self._r_limits[1])

        self._finger_l_motor.setPosition(position_left)
        self._finger_r_motor.setPosition(position_right)

        pos_msg = Float32()
        # normalize to [0, 1]:
        pos_msg.data = (self._finger_r_sensor.getValue() - self._r_limits[1]) / (self._r_limits[0] - self._r_limits[1])
        self._state_publisher.publish(pos_msg)
