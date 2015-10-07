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
        export ROS_WORKSPACE=~/catkin_ws

3. Make clone of the Raspberry Pi user land programs source code:

        cd ~
        git clone https://github.com/raspberrypi/userland.git

4. Clone build and install Raspberry Pi firmware updater:

        cd ~
        sudo curl -L --output /usr/bin/rpi-update https://raw.githubusercontent.com/Hexxeh/rpi-update/master/rpi-update && sudo chmod +x /usr/bin/rpi-update
        sudo rpi-update

5. Build the Raspberry Pi userland programs:

        cd ~/userland
        ./buildme

6. Install this respository:

        cd ~/catkin_ws/src
        git clone https://github.com/UbiquityRobotics/raspicam_node.git

7. Make sure the image `compress-image-transport` library is present:

        sudo apt-get ros-indigo-compress-image-transport

8. Build everything:

        cd ~/catkin_ws
        catkin_make)

9. Make `/dev/vchiq` accessible to regular users (there is probably
   a more secure way to do this, but for now...)

        sudo chmod 666 /dev/vchiq


10. Make sure that catkin workspace is visible to ROS:

        source devel/setup.bash

11. Run the raspicam node:

        roscore &
        rosrun raspicam raspicam_node &
        rosservice call /raspicam_node/camera/start_capture 

12. View the image on a laptop/desktop:

        rostopic list
        rqt_image_view image:=/raspicam/camera/image/compressed


## TO DO List :

* remove warnings from raspicamcontrol
* reenable vc_gencmd

## Old text that will eventually be deleted

5. Install additional groovy packages:

        sudo apt-get install ros-groovy-image-transport ros-groovy-image-transport-plugins
        sudo apt-get ros-groovy-image-transport-plugins ros-groovy-camera-info-manager

1. Get Raspbian Wheezy from
   [http://elinux.org/RPi_Easy_SD_Card_Setup](http://elinux.org/RPi_Easy_SD_Card_Setup).

2. Enable Camera and expand FS by reading
   [http://www.raspberrypi.org/archives/3890](http://www.raspberrypi.org/archives/3890)

3. Upgrade the system to the latest:

        sudo apt-get update
        sudo apt-get upgrade

4. Install ROS
   [Groovy](http://www.ros.org/wiki/groovy/Installation/Raspbian].

