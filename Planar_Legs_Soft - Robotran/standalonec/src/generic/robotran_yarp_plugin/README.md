Using Yarp with Robotran
=====================================

 General information 
---------------------------

Installation of Yarp 
-------------------------------

See [icub wiki] (http://wiki.icub.org/yarp/specs/dox/user/html/install.html) for installation instruction details.


* _sudo apt-get install libboost-all-dev_
* _sudo apt-get install cmake libace-dev git_
* _git clone git://github.com/robotology/yarp.git_
* _cd yarp && mkdir build && cd build && cmake .. && make_

* _sudo make install_
* _sudo ldconfig_

Running simulation with Yarp
-----------------------------------
To indicate the location of the robot configuration files, run the following in the terminal :

```
export YARP_ROBOT_NAME=walkman
```
In the build folder : 

```
export YARP_DATA_DIRS=./share/robotran:Path_to/robotranyarpplugins/build/share/robotran
```
