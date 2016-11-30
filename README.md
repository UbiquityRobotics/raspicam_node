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

* `/camera/camera_info`:
  Publishes `sensor_msgs/CameraInfo` camera info for each frame.

Services:

* `/camera/start_capture`:
  Start video capture and publication. 
  NOTE ATP42 - this has been removed we always publish.

* `/camera/stop_capture`:
  Stop video capture and publication (buggy at the moment).

* `/set_camera_info`:
  Set camera information (used for calibration).
  Saved in `package://raspicam/calibrations/camera.yaml` .

Parameters:

* `camera_frame_id`: The frame identifier to associate the camera.

* `camera_info_url`: The URL of the camera information `.yaml` file.

* `camera_name`: The name of the camera.

* `framerate` (default: 30): Framerate of the captured images
   (0 < framerate <= 90).

* `height`: Height of the captured images (0 < width <= 1080).

* `quality`: Quality of the captured images (0 < quality <= 100).

* `srrc_publishing_mode`: 

   * `0`: Publish images always when the camera is on (default).

   * `1`: Publish only one image by request.

   * `2`: Publish one image and then switch to waiting mode.  (Huh?)

* `tf_prefix`: Prefix for `camra_frame_id`.

* `width`: Width of the captured images (0 < width <= 1920).

Parameter changes are applied after the camera is stoped
using the `/stop_capture` and then restarted using the
`/start_capture` service.

Example 1:

        rosrun raspicam raspicam_node
        #rosservice call /camera/start_capture
        rosrun image_view image_view image:=/camera/image _image_transport:=compressed

If you want to try 90 fps mode, you'll have to decrease the quality factor.
To try the 90 fps mode :

        rosrun raspicam raspicam_node _framerate:=90 _quality:=10
        #rosservice call /camera/start_capture
        rosrun image_view image_view image:=/camera/image _image_transport:=compressed

## Build Intructions

1. Log into a Raspberry Pi 2 with
   [Ubuntu and ROS installed](Doc_Downloading_and_Installing_the_Ubiquity_Ubuntu_ROS_Kernel_Image.md).

2. Make clone, build and install the Raspberry Pi user land programs
   from source code:

        cd ~
        git clone https://github.com/raspberrypi/userland.git
        cd ~/userland
        ./buildme

3. Make sure the image `compressed-image-transport` library is present:

        sudo apt-get install -y ros-indigo-compressed-image-transport

4. Make a catkin workspace:

        # The workspace may already have been constructed; if so skip this step:
        cd ~
        mkdir -p catkin_ws/src
        source /opt/ros/indigo/setup.bash
        #export ROS_WORKSPACE=~/catkin_ws

5. Install a couple of Ubiquity Robotics repositories:

        cd ~/catkin_ws/src
        git clone https://github.com/UbiquityRobotics/raspicam_node.git
        # The repository has some useful launch files:
        git clone https://github.com/UbiquityRobotics/ubiquity_launches.git

6. Build everything:

        cd ~/catkin_ws
        catkin_make

7. Make `/dev/vchiq` is accessible to users in video group:

        sudo -s
        echo 'SUBSYSTEM=="vchiq",GROUP="video",MODE="0660"' > /etc/udev/rules.d/10-vchiq-permissions.rules
        usermod -a -G video `whoami`
        reboot

8. Make sure that catkin workspace is visible to ROS:

        # This may already be in your `!~/.bashrc` file:
        source ~/catkin_src/devel/setup.bash

9. Clone, build, install, and run Raspberry Pi firmware updater:

        cd ~
        sudo apt-get install -y curl
        sudo curl -L --output /usr/bin/rpi-update https://raw.githubusercontent.com/Hexxeh/rpi-update/master/rpi-update && sudo chmod +x /usr/bin/rpi-update
        #sudo rpi-update cad980c560b6c240fdaf6cb4b7703921b18114e3
        sudo rpi-update
        sudo reboot

10. Run the raspicam node:

        roscore &
        rosrun raspicam raspicam_node &
        #rosservice call /raspicam_node/camera/start_capture 

    If you get an error that looks like:

        mmal: mmal_component_create_core: could not find component 'vc.ril.camera'
        mmal: Failed to create camera component
        [INFO] [1444257925.655477127]: init_cam: Failed to create camera component
        Segmentation fault

    The two things to check are to 1) make sure the Raspberry Pi
    firmware as been updated and 2) make sure the cable is properly
    seated.  Both these issues generate the same error message.
    If you still get this error, you probably have a bad version of
    Raspberry Pi firmware.  Try to find older firmware version to feed
    into the rpi-update` command.  The firmware version number is stored
    in `/boot/.firmware_revision`.

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
        
## Notes:

Frankly, there was an element of luck required to figure out
how to get the Raspberry Pi camera working under Ubuntu.
The reason why is described below.

The raspicam node uses the Raspbery Pi video camera interface
API (Application Programming Interface) to access the image data.
The Raspberry Pi camera is actually connected to the GPU (Graphics
Processing Unit) where specalized graphics processing units
can manipulate the image before it is forwarded on to the
main ARM7 cores.  The Raspberry Pi GPU is closed source
proprietary code is only shipped as a binary blob that is
loaded into the GPU at processor boot up time.

In general, the Raspberry Pi camera is fully supported by
the Raspberry Pi foundation.  However, in order to support
their older products (the model A, model B, and model B+),
they have had to develop (with user help) their own Linux
distribution called Raspian.  The Raspberry Pi camera is
fully supported by the Raspian Linux distribution.
ROS is only supported by the Ubuntu Linux distribution.
Since the `raspicam_node` is for ROS, it must be run on
a Ubuntu Linux distribution.  While the Ubuntu Linux distribution
is not fully supported by the Raspberry Pi Foundation,
the Raspberry Pi foundation is actually OK with the Ubuntu
Linux distribution and does provide some limited support.

Both the Ubuntu and Raspian distributions use the same
packaging system called Debian Packages.  To make things
confusing, the Raspian Linux distribution is based on the
Debian Linux distribution.  While the debian package formats
are the same between both the Ubuntu and Raspian Linux
distributions, the underlaying Debian packages are *NOT*
100% interoperable.  Sometimes the Ubuntu Linux distribution
installs files in a different location than the Raspian
distribution does.  This is great fun.

There are two ways that firmware is provided to the Raspberry Pi.
There is a package called `raspberrypi-bootloader-nokernel` which
provides the binary blob *and* there is a program called `rpi-update`
that can do so as well.  The bootloader stuff tends to be older
than the `rpi-update` method.

After compiling `userland` using the `buildme` script, the
`raspicam_node` code compiles without errors.  When the
`raspicam_node` was first run it failed.  After running the
`rpi-update` program it worked for a while.  As other people
tryed to install `raspicam_node`, it simply did not work.
After much sluething it became clear that the Raspberry Pi
foundation had accidently checked in a firmware version that
did not work.

Our temporary solution is to use slightly older firmware,
until whatever problem that caused `raspicam_node` to not work,
to start working again.

It was a small miracle is that there was a small window were
we figured out the firmware worked, and we managed to find that
window.  Once we figured out that it was firmware, we were on
solid ground to finding a work around. Initially we used the
command below:

        sudo rpi-update cad980c560b6c240fdaf6cb4b7703921b18114e3

to install a specific firmware that worked.  These days, the
latest released firmware version is used, because it works.
So, we no longer explictly specify the firmeware version.

If you are curious about firmware revision numbers, there is a
[post](http://raspberrypi.stackexchange.com/questions/29991/how-do-i-find-the-firmware-repository-commit-which-matches-the-firmware-version)
about them.

## TO DO List :

* remove warnings from raspicamcontrol
* reenable vc_gencmd

## Calibration

The raspicam_node package contains a calibration file for the raspberry
PI camera versions 1 and 2.  The procedure for calibrating a camera is only necessary
if you change the lens or image size used.

A tutorial 
  [Monocular Camera Calibration tutorial](http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration)
shows how to calibrate a single camera.

The
  [8x6 checkerboard](http://wiki.ros.org/camera_calibration/Tutorials/StereoCalibration?action=AttachFile&do=view&target=check-108.pdf)
and the
  [7x6 checkerboard](http://wiki.ros.org/camera_calibration/Tutorials/StereoCalibration?action=AttachFile&do=view&target=check_7x6_108mm.pdf)
are rather large and require specialized printers to print out at
full scale.  They can be printed on more common printer sizes
with auto scaling turned on.  Be sure to carefully measure the
square size in millimeters and convert to meters by dividing by 1000.


A camera calibration can be produced with the following commands:

    $ rosrun image_transport republish compressed in:=/raspicam_node/image raw out:=/raspicam_node/image
    $ rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.74 image:=/raspicam_node/image camera:=/raspicam_node

