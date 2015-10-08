# raspicam_node

ROS node for camera module of Raspberry Pi

Now works at 90 fps thanks to the new firmware provided by
the Raspberry Pi foundation

## Node Information

Requirements:

* working ROS core and network

* a Raspberry Pi

* a Raspberry Pi camera module

Topics:

* `/camera/compressed`:
  Publishes `sensor_msgs/CompressedImage` with jpeg from the camera module.

* `camera/camera_info`:
  Publishes `sensor_msgs/CameraInfo` camera info for each frame.

Services:

* `/camera/start_capture`:
  Start video capture and publication.

* `/camera/stop_capture`:
  Stop video capture and publication (buggy at the moment).

* `/set_camera_info`:
  Set camera information (used for calibration).
  Saved in `package://raspicam/calibrations/camera.yaml` .

Parameters:

* `width`: Width of the captured images (0 < width <= 1920).

* `height`: Height of the captured images (0 < width <= 1080).

* `framerate`: Framerate of the captured images (0 < framerate <= 90).

* `quality`: Quality of the captured images (0 < quality <= 100).

* `tf_prefix`: Prefix for frame_id.

For parameter changes to be applied, the capture need to be restarted
using the `/stop_capture` and `/start_capture` services.

Example 1:

        rosrun raspicam raspicam_node
        rosservice call /camera/start_capture
        rosrun image_view image_view image:=/camera/image _image_transport:=compressed

If you want to try 90 fps mode, you'll have to decrease the quality factor.
To try the 90 fps mode :

        rosrun raspicam raspicam_node _framerate:=90 _quality:=10
        rosservice call /camera/start_capture
        rosrun image_view image_view image:=/camera/image _image_transport:=compressed

## Build Intructions

1. Log into a Raspberry Pi 2 with
   [Ubuntu and ROS installed](Doc_Downloading_and_Installing_the_Ubiquity_Ubuntu
_ROS_Kernel_Image.md).

2. Make a catkin workspace:

        cd ~
        mkdir -p catkin_ws/src
        source /opt/ros/indigo/setup.bash
        #export ROS_WORKSPACE=~/catkin_ws

3. Clone, build, install, and run Raspberry Pi firmware updater:

        cd ~
        sudo apt-get install -y curl
        sudo curl -L --output /usr/bin/rpi-update https://raw.githubusercontent.com/Hexxeh/rpi-update/master/rpi-update && sudo chmod +x /usr/bin/rpi-update
        sudo rpi-update

4. Make clone, build and install  the Raspberry Pi user land programs
   from source code:

        cd ~
        git clone https://github.com/raspberrypi/userland.git
        cd ~/userland
        ./buildme

5. Install a couple of Ubiquity Robotics repositories:

        cd ~/catkin_ws/src
        git clone https://github.com/UbiquityRobotics/raspicam_node.git
        # The repository has some useful launch files:
        git clone https://github.com/UbiquityRobotics/ubiquit_launch.git

6. Make sure the image `compressed-image-transport` library is present:

        sudo apt-get install ros-indigo-compressed-image-transport

7. Build everything:

        cd ~/catkin_ws
        catkin_make

8. Make `/dev/vchiq` is accessible to users in video group:

        sudo -s
        echo 'SUBSYSTEM=="vchiq",GROUP="video",MODE="0660"' > /etc/udev/rules.d/10-vchiq-permissions.rules
        usermod -a -G video `whoami`
        exit

9. Make sure that catkin workspace is visible to ROS:

        source devel/setup.bash

10. Run the raspicam node:

        roscore &
        rosrun raspicam raspicam_node &
        rosservice call /raspicam_node/camera/start_capture 

11. View the image on a laptop/desktop:

        rostopic list
        rqt_image_view image:=/raspicam_node/camera/image/compressed

12. To use the launch files:

        export PATH=$PATH:~/catkin_ws/src/ubiquity_launches/bin
        # On Robot, do either:
        loki_raspicam   # ~30 FPS
        # or do:
        loki_raspicam90  # 90 FPS
        # On laptop/desktop:
        loki_view_raspicam
        
## TO DO List :

* remove warnings from raspicamcontrol
* reenable vc_gencmd

