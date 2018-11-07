^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package raspicam
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.4.0 (2018-11-07)
------------------
* Add support for optionally publishing raw image
* Add parameter for selecting the camera id (Compute Module)
* Don't allocate new messages for every frame
* Cleanup log noise
* Fix gcc warnings (enable -Wall)
* Fix dynamic reconfigure bug (Fixes `#31 <https://github.com/UbiquityRobotics/raspicam_node/issues/31>`_)
* Remove old unused cruft
* Use unique_ptr with custom deleters to manage mmal components
* Pass state around by reference instead of as a global
* Use C++14
* Use clang format from https://github.com/davetcoleman/roscpp_code_format
* Contributors: Jose Eduardo Laruta Espejo, Rohan Agrawal

0.3.0 (2018-09-28)
------------------
* use user writeable calibration after the package one
* set camera name everywhere
* Contributors: Rohan Agrawal

0.2.3 (2018-09-20)
------------------
* new calibration parameters
* Contributors: Rohan Agrawal

0.2.2 (2017-11-19)
------------------
* add proper includes to header RaspiCamControl
* applied clang format
* Add low res mode for tracking
* Add note about publish rate over the network.
* Add note about dynamic reconfigure
* Contributors: Jan Koniarik, Jim Vaughan, Rohan Agrawal, davecrawley

0.2.1 (2017-02-25)
------------------
* Dependency fixes
* Contributors: Rohan Agrawal

0.2.0 (2017-02-25)
------------------
* Add launch files and camera calibrations
* Add documentation to get pi-deps
* Proper dependancy declaration
* Pull in all SV-ROS development
* Dynamic reconfigure support
* Support for both CameraV1 and V2
* Contributors: Girts Linde, Jim Vaughan, Rohan Agrawal

0.1.0 (2016-04-14)
------------------
* Fixed link order bug
* Make structs char const instead of char
  Prevents lots of compile warnings
* Cleanup package.xml, using format 2
* Move find_lib stuff to more appropriate place in CMakeLists
* More robust CMakeLists
* Got raspicam to compile a fake node on the x86_64 architecture without any modification to CMakelists.txt
* Initial commit
* Contributors: David Datjko, Girts Linde, Jim Vaughan, Kent Williams, Rohan Agrawal, Wayne C. Gramlich, atp42, datjko, fpasteau
