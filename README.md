# Semi-Dense Cross-Modal 6-DoF Tracking of an Event Camera for Challenging Conditions

This paper is a novel pipline for cross-modal 6-DoF tracking with an event camera. Because the paper is not published, we only release the compiled file here. We will release the source code when the paper is accepted.

### Related Publications

* **[DEVO: Depth-Event Camera Visual Odometry in Challenging Conditions](https://arxiv.org/pdf/2202.02556.pdf)**, Yi-Fan Zuo, Jiaqi Yang, Jiaben Chen, Xia Wang, Yifu Wang and Laurent Kneip, ICRA 2022.

# 0. Structure

We provide 4 systems (Canny-DVEO, Canny-EVT, Canny-EVT-- and Canny-VT) in different branches. The main branch is Canny-EVT Every branch is a workspace. We will indiscriminately call these four branches as catkin_ws in this document.

# 1. Installation

We have tested the system on Ubuntu 18.04.5 LTS + ROS melodic + OpenCV 3

## 1.1 Dependencies Installation

Dependencies are specified in the file [dependencies.yaml](dependencies.yaml). They can be installed with the following commands from the `src` folder of your catkin workspace:

~~~
$ cd ~/catkin_ws/src/EVENT-D-VO  
$ sudo apt-get install python3-vcstool
$ vcs-import < dependencies.yaml
~~~

You also can git clone the dependencies according to the file [dependencies.yaml](dependencies.yaml) and install them.



You may need `autoreconf` to compile glog_catkin. To install `autoreconf`, run


	$ sudo apt-get install autoconf

Note that above command may change on different version of Ubuntu. 
Please refer to https://askubuntu.com/a/269423 for details.

**yaml-cpp** is only used for loading calibration parameters from yaml files:

	$ cd ~/catkin_ws/src/EVENT-D-VO 
	$ git clone https://github.com/jbeder/yaml-cpp.git
	$ cd yaml-cpp
	$ mkdir build && cd build && cmake -DYAML_BUILD_SHARED_LIBS=ON ..
	$ make -j



## 1.2 Our systems Installation

	$ cd ~/catkin_ws(choose a system you want to build)/ 
	$ catkin_make
	$ source /devel/setup.bash

# 2. Usage
To run the pipeline, you need to download rosbag files from the [VECtor Page](https://star-datasets.github.io/vector/).

Please note that the code is the compiled file. You cannot see all the source code.

This code is not the release version, the Canny-EVT may not run in real-time.

## 2.1 Mapping
We provide some semidense.pcd documentaries in Semidense_Map/VECtor dirctory. You can use the map to run Canny-EVT, Canny-EVT-- and Canny-VT directly.

## 2.2 Our system

Choose a system you want to run. Revise the src/esvo_plus/cfg/system_mpl_vector.yaml. We provide the start_time_list to find the start_time, which can make sure you get the correct initial pose.

    $ cd ~catkin_ws/

For Canny-EVT,  run,

    $ roslaunch canny_evt system_slam_vec.launch

For Canny-EVT--, run,

~~~
$ roslaunch canny_evt_minus system_slam_vec.launch
~~~

For Canny-VT, run,

~~~
$ roslaunch canny_vt system_slam_vec.launch
~~~

For Canny-DEVO, run 

~~~
$ roslaunch canny_devo system_vec.launch
~~~

For visualization, please check the topic: (package_name)/repj and (package_name)/ts.
