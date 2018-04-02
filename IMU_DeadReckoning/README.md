# IMU Dead Reckoning
The first assignment for Self-Driving Cars, 2018 Spring, NCTUEE.

## Implemantation Details
Pure IMU integration to	get	the	pose of	the	IMU	for	each new accelerometer and gyroscope 
measurements
- [Direction Cosine Matrix](http://www.starlino.com/dcm_tutorial.html)
- [Chapter 6.1, 6.2](https://www.cl.cam.ac.uk/techreports/UCAM-CL-TR-696.pdf)

Download the IMU data [here](https://drive.google.com/file/d/1hmGRmqLOlDLXahHyDBTpedyqZl6V-Byw/view)

## Getting Started
These instructions will get you a copy of the project up and running on your local machine for development and testing purposes.

### Prerequisites
You will need to do matrix operation during implementation: Use Eigen3
Note: It is already installed when installing ROS
If not, you can install Eigen3:
```
sudo apt-get install libeigen3-dev
```
- [Eigen3 Tutorial](http://eigen.tuxfamily.org/dox-devel/group__QuickRefPage.html)
- For python, see [here](https://github.com/jrl-umi3218/Eigen3ToPython)

## Running the test
How to run the automated tests for this system

### Break down into steps
```
`roscore`
rosrun IMU_deadReckoning deadReckoning.py
rosbag play "bag's_name"
rviz
```

### Result
![Result](https://github.com/simon10030950/Self-Driving-Car/blob/master/IMU_DeadReckoning/src/IMU_deadReckoning/result.png)

## Built With
- [ROS](http://www.ros.org/)
- [Python3](https://www.python.org/download/releases/3.0/)
- [Eigen3](http://eigen.tuxfamily.org/index.php?title=Main_Page)

## Authors
- **Simon Hsieh** - *initial work* - [simon10030950](https://github.com/simon10030950)



