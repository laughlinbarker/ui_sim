ui_sim meta package
===================

This repository contains all the ROS packages necessary to run the Under-Ice UUV
simulation in Gazebo, as described in [1].

**Disclaimer:** this is research-grade code! While the mathematical concepts employed are (hopefully!) sound, the implenetation includes more hacks than is ideal. I further acknowledge that it is poor practice to bundle multiple discrete (and maintained!) software packages into a single repository, but as I'm currently short on time, I thought it more prudent to simply publish early! In the event I have the bandwidth (and perhaps interest from others), I will work to disentangle things.

#### Discrete ROS packages contained in this repository:
- uuv_simulator (meta package)
- hector_gazebo
- laughlin_uw_plugins
- tdma_broadcaster
- under_ice_sim


### Install External Deps

For UUV_Simulator meta package (per UUV_Simulator Wiki):
```
sudo apt-get install ros-kinetic-gazebo-msgs ros-kinetic-gazebo-plugins ros-kinetic-gazebo-ros ros-kinetic-gazebo-ros-control ros-kinetic-gazebo-ros-pkgs ros-kinetic-effort-controllers ros-kinetic-image-pipeline ros-kinetic-image-common ros-kinetic-perception ros-kinetic-perception-pcl ros-kinetic-robot-state-publisher ros-kinetic-ros-base ros-kinetic-viz python-wstool python-catkin-tools python-catkin-lint ros-kinetic-hector-localization ros-kinetic-joy ros-kinetic-joy-teleop libopencv-dev protobuf-compiler protobuf-c-compiler ros-kinetic-gps-common
```
For trajectory controller in UUV_Simulator (the trajectory tracker wants numpy and scipi, don't recall where/how I landed on installing other packages)
```
sudo apt-get install python-numpy python-scipy python-matplotlib ipython ipython-notebook python-pandas python-sympy python-nose
```
Install GeographicLib (I use this in my modified GPS plugin for better cartesian-WGS84 ellipsoid conversion):
```
cd ~/ros_ws/src/under_ice_meta
tar -zxvf GeographicLib-1.48.tar.gz
cd GeographicLib-1.48
mkdir build
cd build/
cmake ..
make
sudo make install
```
## Build and run

My package deps are not clean yet, so build my plugin package first
```
roscd 
cd ..
catkin_make --pkg laughlin_uw_plugins
catkin_make
```

### Define a few paths 

Prior to launching the simulation, you need to **modify three paths** defined in 
`prepExpResultsFolder.sh`. This script is run with `ice.launch` and creates a c
onsistant folder structure, where bag files are stored for later use, as well as copies of all ice, vehicle, etc. xacro files.
This can be useful for determining, "What ice velocity did I use in that experiment?..."

Suppose you wish to log data to `/sim_data`

(neglecting folder permissions ownership, yadda yadda)
```
mkdir /sim_data 
```

Now change the following three paths to reflect your desired save location, and 
full path to `.../under_ice_sim/models/...`

```
roscd under_ice_sim/scripts/
vim prepExpResultsFolder.sh
```

Change the following paths to reflect where you wish to save data, and location 
of the ice and vehicle xacro files:

```
save_path="/sim_data"
ice_model_path="/home/laughlin/ros_ws/src/under_ice_sim/models/ice"
veh_model_path="/home/laughlin/ros_ws/src/uuv_simulator/uuv_descriptions/models/rexrov/urdf"
```

Lastly, change the default path where rosbags are to be saved by editing `data_record.launch`
to reflect the appropriate paths (as above):
```
roscd under_ice_sim/launch
vim data_record.launch
```
Change each of the three paths to read `args="--output-name /sim_data/$(arg folder)/bags/measurements.bag` ...

Prior to launching a simulation, create a folder within `/sim_data` to store this simulation's results:
```
mkdir /sim_data/simulation1
```

To launch the simulation, we will launch 6 distinct launch files. They should
be launched in this order, so use Terminator, tmux, or screen to give yourself 
at least six prompts:

``` 
roslaunch under_ice_sim empty_underwater_world.launch timeout:=1000
roslaunch uuv_descriptions upload_rexrov_ui.launch x:=1050 y:=0 z:=-30 yaw:=3.14
roslaunch under_ice_sim ice.launch folder:=simulation1
roslaunch under_ice_sim vehicle_pid_controller.launch
roslaunch under_ice_sim send_waypoints_lawnmower_1000x100_6_10.launch
roslaunch under_ice_sim data_record.launch folder:=simulation1
```

**Note:** 
1. By default Gazebo is run in a headless mode to reduce CPU load. RVIZ
can be used for visualization. To change this, modify the appropriate line in 
`empty_underwater_world.launch`

If you wish to run the EKF you will need to do a few things as well.

Edit `ui_ekf/launch/defaultEKF.launch`  to reflect:

```
<arg name="output_path" value="/sim_data/$(arg sim_folder)/$(arg output_folder)" />
```

Create and experiment folder where EKF results will be saved (I create a seperate
folder for each "experiment" I do):
```
mkdir /sim_data/simulation1/ekf_test01
```

Copy the default EKF config file to the experiment folder (this contains all the parameters needed by the EKF)
```
roscd ui_ekf/config
cp default.yaml /sim_data/simulation1/ekf_test01
```

The EKF should now be ready to run:
```
roslaunch ui_ekf defaultEKF.launch sim_folder:=simulation1 output_folder:=ekf_test01
```

To unpause Gazebo from the command line:
```
gz world -p 0
```

Laughlin Barker, January 17, 2018

[1] Barker, L. D. L., and Whitcomb, L. L.,"A Preliminary study of ice-relative undwater vehicle navigation beneath moving sea ice," IEEE International Conference on Robotics and Automation (ICRA), 2018. Accepted.
