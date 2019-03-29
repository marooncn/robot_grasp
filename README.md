## Introduction
This package contains only the simulation environment, the control code can be found [here](https://github.com/alxer/JCAR-Competition).
![](https://github.com/marooncn/robot_grasp/blob/master/img/ezgif.com-optimize%20(2).gif)
## Run the demo by yourself
Tested on Ubuntu 16.04 (64 bits) and ROS version is Kinetic. <br>
Dependence: [kinova-ros](https://github.com/Kinovarobotics/kinova-ros) <br>
0> Compile
~~~
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/Kinovarobotics/kinova-ros.git 
sudo apt-get install ros-kinetic-gazebo-ros* 
sudo apt-get install ros-kinetic-gazebo-ros-control
sudo apt-get install ros-kinetic-ros-controllers*
git clone https://github.com/marooncn/robot_grasp.git
cd ..
catkin_make
~~~
1> Load the grasp model  
~~~
source ~/catkin_ws/devel/setup.bash
roslaunch robot_grasp load_model.launch
~~~
Then snap kinect to kinect_support with face down. <br>
2> Load the grasp objects <br>
2.1 > Insert the grasp object one by one
~~~
cp -r ~/catkin_ws/src/robot_grasp/models/grasp_object/* ~/.gazebo/models/
~~~
Then you can insert the corresponding object under Gazebo 'Insert' toolbar. <br>
2.2> Insert the grasp by roslaunch
~~~
roslaunch robot_grasp load_object.launch
~~~
<img alt="grasp model" src="https://github.com/marooncn/robot_grasp/blob/master/img/simulation.jpg" width="800">
3> Tune the PID parameters <br>
The robot model is controlled by PID. If the model is unstable, then you can change the PID parameters until it's stable. To do this, open 'rqt', then choose Plugins -> Configuration -> Dynamic Reconfigure -> j2s7s300  <br>
4> Kinect Information <br>
Open 'rviz' and Set Fixed Frame to camera_link，then Add-> By topic -> points -> PointCloud2 to show its pointcloud information and you can also check the rgb image.
<img alt="rviz" src="https://github.com/marooncn/robot_grasp/blob/master/img/rviz.png" width="800">

## Reference
[Use a Gazebo Depth Camera with ROS](http://gazebosim.org/tutorials?tut=ros_depth_camera&cat=connect_ros#View%20Depth%20Camera%20Output%20in%20RViz) <br>
[jaco_on_table](https://github.com/JenniferBuehler/jaco-arm-pkgs/tree/master/jaco_tutorial/jaco_on_table) <br>
[Tutorial: Using roslaunch to start Gazebo, world files and URDF models](http://gazebosim.org/tutorials?tut=ros_roslaunch)
