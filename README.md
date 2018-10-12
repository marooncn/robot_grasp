## Introduction
    This package contains grasp model.

## Run the demo by yourself
Dependence: [kinova-ros](https://github.com/Kinovarobotics/kinova-ros) <br>
0> Compile
~~~
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/Kinovarobotics/kinova-ros.git -b release-1.2
git clone https://github.com/marooncn/robot_grasp.git -b v1
cd ..
catkin_make
~~~
1> Load the grasp model  
~~~
source ~/catkin_ws/devel/setup.bash
rosrun robot_grasp load_model.launch
~~~
2> Insert the grasp objects
~~~
cp -r ~/catkin_ws/src/robot_grasp/models/grasp_object/* ~/.gazebo/models/
~~~
Then you can insert the corresponding object under Gazebo 'Insert' toolbar, or insert them automatically
~~~
rosrun robot_grasp load_object.launch
~~~
![](https://github.com/marooncn/robot_grasp/blob/v1/img/model.png)
## Reference
[Use a Gazebo Depth Camera with ROS](http://gazebosim.org/tutorials?tut=ros_depth_camera&cat=connect_ros#View%20Depth%20Camera%20Output%20in%20RViz) <br>
[table model file](https://github.com/JenniferBuehler/jaco-arm-pkgs/tree/master/jaco_tutorial/jaco_on_table)
