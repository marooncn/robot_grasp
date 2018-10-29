# robot_grasp
    This repository saves some code of JCAR Compedtition. The competition includes simulation part and 
    physical part. branch v2 is the simulation environment I build. Thank Jianbo Zhang to build 
    3D model files. This branch includes classification and cooridinate transform code of physical part.
    
## [Simulation part](https://github.com/marooncn/robot_grasp/tree/v2)
<img alt="" src="https://github.com/marooncn/robot_grasp/blob/master/img/simulation.jpg" width="600"> <br>
We improve the [official](https://github.com/Kinovarobotics/kinova-ros) kinova model, [here](https://github.com/Kinovarobotics/kinova-ros/issues/157) are the details.
## Physical part
### environment
<img alt="" src="https://github.com/marooncn/robot_grasp/blob/master/img/physical.jpg" width="600"> <br>
### classify
raw image from kinect <br>
<img alt="raw image from kinect" src="https://github.com/marooncn/robot_grasp/blob/master/img/1.png" width="600"> <br>
grayscale <br>
<img alt="grayscale" src="https://github.com/marooncn/robot_grasp/blob/master/img/2.png" width="600"> <br>
classification with decision tree <br>
<img alt="classification with decision tree" src="https://github.com/marooncn/robot_grasp/blob/master/img/Image.png" width="600"> <br>
### coordinate transformation
then transfrom the goal point in the captured image into space point in the robot base cooriedinator. <br>
python version: [transform.py](https://github.com/marooncn/robot_grasp/blob/master/scripts/transform.py) <br>
C++ version: [transform.cpp](https://github.com/marooncn/robot_grasp/blob/master/src/transform.cpp)
