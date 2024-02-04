# Running Example 
## Linux Machine:

install webots for linux:
https://cyberbotics.com/doc/guide/installation-procedure?tab-os=linux

first terminal:
> webots clair_webots/worlds/ur_scratch.wbt

second terminal:
> export WEBOTS_HOME=/usr/local/webots
> 
> $WEBOTS_HOME/webots-controller clair_webots/controllers/UR5e_controller.py 


## Windows and WSL:
When using WSL, it is better (and faster) to run the webots instance on windows.
You should install webots for windows and you can run it from the wsl as described below.
Of course this whole process could be done directly from windows without using WSL, but
it's a good idea to make sure that everything works that way before moving to ROS where Linux
must be used.

first terminal:
> export WEBOTS_HOME="/mnt/c/Program Files/Webots"
> 
> "$WEBOTS_HOME/msys64/mingw64/bin/webots.exe" clair_webots/worlds/ur_scratch.wbt

second terminal:
> export WEBOTS_HOME="/mnt/c/Program Files/Webots"
> 
> "$WEBOTS_HOME/msys64/mingw64/bin/webots-controller.exe" clair_webots/controllers/UR5e_controller.py 


-----

# Running example with two robots, one with a camera:
In the following example the 'export WEBOTS_HOME="/mnt/c/Program Files/Webots"' is omitted. 
It is assumed that the environment variable is already set.

Run the following world file:
> "$WEBOTS_HOME/msys64/mingw64/bin/webots.exe" clair_webots/worlds/ur_gripper_and_camera.wbt

In this world file there are two robots that are waiting for an external controller. We have to run the
two controllers in separate terminals, and pass the name of the robot to each controller. UR5e_1 is the robot
with the gripper and UR5e_2 is the robot with the camera.

In two separate terminals run the following commands:

>"$WEBOTS_HOME/msys64/mingw64/bin/webots-controller.exe" --robot-name="UR5e_1" clair_webots/controllers/UR5e_controller_with_ik.py

>"$WEBOTS_HOME/msys64/mingw64/bin/webots-controller.exe" --robot-name="UR5e_2" clair_webots/controllers/UR5e_camera_controller.py

The first controller does the same as in the simple example: pick up the box and drop it. The second controller
moves the robot with the camera a little bit, and takes a picture few times a second. When it finishes, it plots
an animation from the 200 images it took. 

*Note: The simulation may run much slower when recording the images continuously (It may not reach the 1x speed).*

Since it may be complicated to run 3 terminals at the same time, you can create script to run all the commands at once.
An example is the file "two_robots_camera_example.sh" in the root of the repository.
