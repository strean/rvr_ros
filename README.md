
An experiment to write a ROS node for the Sphero RVR.

Currently a very rough base_controller that responds to cmd_vel/Twist. I'm able to control the RVR with teleop_twist_keyboard. Testing on a Jetson Nano running ubuntu 18.04. 


Based on Rud Merriam's "RVR CPP" https://bitbucket.org/rmerriam/rvr-cpp/src/master/

Assumes rvr++ header files are placed in include/rvr++ and rvr++ is compiled to a static library and placed in lib. I've included a Makefile that works on my device.


