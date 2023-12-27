def print_devices_names(robot):
    n_devices = robot.getNumberOfDevices()
    for i in range(n_devices):
        device = robot.getDeviceByIndex(i)
        # print device name and whether it is a motor or a sensor
        print(device.getName())


def get_motor_list(robot, motor_ids):
    return [robot.getDeviceByIndex(i) for i in motor_ids]

