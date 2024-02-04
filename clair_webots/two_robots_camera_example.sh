"$WEBOTS_HOME/msys64/mingw64/bin/webots.exe" clair_webots/worlds/ur_gripper_and_camera.wbt &
"$WEBOTS_HOME/msys64/mingw64/bin/webots-controller.exe" --robot-name="UR5e_1" clair_webots/controllers/UR5e_controller_with_ik.py &
"$WEBOTS_HOME/msys64/mingw64/bin/webots-controller.exe" --robot-name="UR5e_2" clair_webots/controllers/UR5e_camera_controller.py