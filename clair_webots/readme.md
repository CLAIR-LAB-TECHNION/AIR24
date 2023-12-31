# Linux Machine:

install webots for linux:
https://cyberbotics.com/doc/guide/installation-procedure?tab-os=linux

first terminal:
> webots clair_webots/worlds/ur_scratch.wbt

second terminal:
> export WEBOTS_HOME=/usr/local/webots
> 
> $WEBOTS_HOME/webots-controller clair_webots/controllers/UR5e_controller.py 


# Windows and WSL:
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

