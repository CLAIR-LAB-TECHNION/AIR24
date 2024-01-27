import matplotlib.pyplot as plt
from robot_interfaces.ur5e_robot_with_camera import UR5eRobotWithCamera
from matplotlib.animation import FuncAnimation


animation_intrval = 0.08  # seconds

if __name__ == '__main__':
    ur5_cam = UR5eRobotWithCamera()

    # wait a few seconds for grasping robot to rotate:
    ur5_cam.simulate_seconds(4)

    # make this robot move slowly:
    ur5_cam.scale_max_velocities(0.02)

    # move the camera a little bit during next steps:
    joint_state_initial = ur5_cam.get_current_config()
    joint_state_initial[0] -= 0.2
    joint_state_initial[2] += 0.3
    ur5_cam.set_target_config(joint_state_initial)

    # move the robot and save image from camera every 0.32 seconds:
    images = []
    for i in range(250):
        ur5_cam.simulate_seconds(animation_intrval)
        images.append(ur5_cam.get_camera_image())

    # TODO: run until simulation ends

    # plot animation:
    def animate(i):
        plt.cla()  # Clear the current axes
        plt.imshow(images[i])
        plt.axis('off')  # Turn off axis
        plt.title(f'Frame {i}')
    fig = plt.figure()
    animation = FuncAnimation(fig, animate, frames=len(images), interval=animation_intrval * 1000)
    plt.show()
