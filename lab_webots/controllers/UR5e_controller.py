from controller import Robot, Supervisor
from math import pi, sin

TIME_STEP = 32


def print_devices_names(robot):
    n_devices = robot.getNumberOfDevices()
    for i in range(n_devices):
        device = robot.getDeviceByIndex(i)
        # print device name and whether it is a motor or a sensor
        print(device.getName())


def move_motors_sequentialy(robot, motors_ids, sensors_ids):
    for mot, sens in zip(motors_ids, sensors_ids):
        motor = robot.getDeviceByIndex(mot)
        sensor = robot.getDeviceByIndex(sens)
        sensor.enable(TIME_STEP)
        for i in range(20):
            motor.setPosition(sensor.getValue() + 0.1)
            robot.step(TIME_STEP)
        for i in range(20):
            motor.setPosition(sensor.getValue() - 0.1)
            robot.step(TIME_STEP)


def get_motor_list(robot, motor_ids):
    return [robot.getDeviceByIndex(i) for i in motor_ids]


UR5e_motor_indices = [0, 2, 4, 6, 8, 10]
UR5e_sensor_indices = [1, 3, 5, 7, 9, 11]

if  __name__ == '__main__':
    robot = Robot()
    print_devices_names(robot)

    move_motors_sequentialy(robot, UR5e_motor_indices, UR5e_sensor_indices)

    # gripper_finger_l = robot.getDevice('ROBOTIQ 2F-140 Gripper::left finger joint')
    # gripper_finger_r = robot.getDevice('ROBOTIQ 2F-140 Gripper::right finger joint')
    #
    # l_max_pos = gripper_finger_l.getMaxPosition()
    # l_min_pos = gripper_finger_l.getMinPosition()
    # r_max_pos = gripper_finger_r.getMaxPosition()
    # r_min_pos = gripper_finger_r.getMinPosition()
    # print(l_max_pos, l_min_pos, r_max_pos, r_min_pos)
    #
    # for i in range(100):
    #     # Use sensor ROBOTIQ 2 F - 140 Gripper right finger joint sensor
    #     for t in range(200):
    #         gripper_finger_l.setPosition(t / 200 * l_max_pos)
    #         gripper_finger_r.setPosition(t / 200 * r_max_pos)
    #         robot.step(TIME_STEP)
    #
    #     for t in range(200):
    #         gripper_finger_l.setPosition(l_max_pos - t / 200 * l_max_pos)
    #         gripper_finger_r.setPosition(r_max_pos - t / 200 * r_max_pos)
    #         robot.step(TIME_STEP)



    # shoulder_joint = robot.getDevice('shoulder_pan_joint')
    # shoulder_sensor = robot.getDevice('shoulder_pan_joint_sensor')
    # shoulder_sensor.enable(TIME_STEP)
    #
    # F = 0.5   # frequency 2 Hz
    # t = 0.0   # elapsed simulation time
    #
    # while robot.step(TIME_STEP) != -1:
    #     position = sin(t * 2.0 * pi * F)
    #     # shoulder_joint.setPosition(position)
    #
    #     # set position to inf to disable position control:
    #     shoulder_joint.setPosition(float('inf'))
    #     shoulder_joint.setVelocity(-3)
    #     t += TIME_STEP / 1000.0
    #
    #     print(shoulder_sensor.getValue())
