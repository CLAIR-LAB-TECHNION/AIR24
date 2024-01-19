from robot_interfaces.ur5e_robot import UR5eRobot


class UR5eRobotWithCamera(UR5eRobot):
    """Interface to ar UR5e robot with a camera"""

    def __init__(self, interval=32, max_velocity_scale=0.5, robot_name=None):
        super().__init__(interval, max_velocity_scale, robot_name)

        # make sure supported camera device exists:
        device_names = [self._robot.getDeviceByIndex(i).getName() for i in range(self._robot.getNumberOfDevices())]
        assert 'kinect color' in device_names, "robot doesn't have a supported camera"

        self._camera = self._robot.getDevice('kinect color')
        self._depth_camera = self._robot.getDevice('kinect range')
        self._camera.enable(32)
        self._depth_camera.enable(32)

        self._camera_node = self._robot_node.getField('toolSlot').getMFNode(0)
        camera_translation = self._camera_node.getField('translation').getSFVec3f()
        self.tool_length = camera_translation[1]
        # we take y translation because we had to rotate the camera in the world. Should have taken z otherwise.
        # for more general solution, we can retrieve the camera rotation and calculate the tool length from it.

        self.robot_step()

    def get_camera_image(self):
        return self._camera.getImageArray()

    def get_camera_depth_image(self):
        return self._depth_camera.getRangeImageArray()

    def get_camera_fov(self):
        return self._camera.getFov()

    def get_camera_resolution(self):
        return [self._camera.getWidth(), self._camera.getHeight()]

    def get_camera_intrinsics(self):
        # TODO
        return None
