# MTRN4230_G21

## About

The following repository shows the pick and place task of a UR5 robot arm in ROS and Gazebo. The project is inspired from the works of [__`GitHub: lihuang3/ur5_ROS-Gazebo`__](https://github.com/lihuang3/ur5_ROS-Gazebo.git). Included is a shape and colour object detection module, using a traditional image processing techniques, as well cartesian path planning using moveIt. Attached to the ur5 robot arm is a Robotiq85 gripper that is able to pick up objects with help from the grasp plugin by [__`GitHub: JenniferBuehler/grasp-execution-pkgs`__](https://github.com/JenniferBuehler/grasp-execution-pkgs).

### Prerequisites
- This project was tested on Ubuntu 16.04.6 LTS with ROS Kinetic
- The project use Python 2.7.12 and requires the use of Numpy, cv2 and tkinter as additional libraries
- Gazebo, Moveit and RViz need to be installed

### Setup

#### Installation
Using the abritrary workspace name 'simulation_ws', dowload the repository to 'simulation_ws/src/'
```bash
cd ~/simulation_ws/src
git clone https://github.com/pbassett1996/MTRN4230_UR5_G21.git
```
To build the code, return to the 'simulation_ws' directy and perform the following,
catkin_make
source devel/setup.bash
export SVGA_VGPU10=0

Note: depending on your version of Gazebo, you may receive the following error "fatal error: ignition/math/Inertial.h: ...". If so then please install the missing libignition math 2 package,
```bash
sudo apt-get install libignition-math2-dev
```
To run the code with ROS and Gazebo, execute the following,
```bash
roslaunch mtrn4230_g21 ur5_world.launch
```
#### Running Pick and Place Task
In one terminal type:
```bash
cd ~/simulation_ws/
source devel/setup.bash
cd src/MTRN4230_UR5_G21/mtrn4230_g21/src/
./Main_4230_G21.py
```
In another terminal type:
```bash
cd ~/simulation_ws/
source devel/setup.bash
cd src/MTRN4230_UR5_G21/mtrn4230_g21/src/trajectory_planning/
./TrajectoryPlanner_4230_G21.py
```

### Break down of programs

#### Main_4230_G21.py
The main operating file of the project. This allows interprocess communication through ROS node messages and objects to allow communication between the Object Detection and Motion planner modules. It also allows for user input using a tkinter based GUI.

#### ObjectDetection_4230_G21.py
The computer vision module of the program. Uses the data received from the ROS kinect sensor to identify different objects based on colour and shape. It returns the coordinates of the objects in the global reference frame for the pick-and-place operation.

#### TrajectoryPlanner_4230_G21_.py
These files control the actuation of the robot arm such that it can achieve point-to-point control using the python library MoveIt. This is the base foundation for what will eventually become a trajectory plan. Significant contributions can be accredited to Huang Zhao (https://github.com/lihuang3/ur5_ROS-Gazebo).

#### URDF and SDF files
A variety of different URDF files are included that allow an array of differnet objects to be substantiated in the Gazebo simulation. These can be attributed to Huang Zhao (https://github.com/lihuang3/ur5_ROS-Gazebo) as the files were obtained from his work on the ur5_ROS_gazebo simulation and then edited to suit the application of this project. The Kinect SDF was also included as some minor modifications were also made to its content to allow it to statically spawn in Gazebo simulation.

## Built With

* [ROS](https://www.ros.org/) - Operating system used
* [Gazebo](http://gazebosim.org/) - Simulation environment used
* [python](https://www.python.org/) - Programming language used
* [OpenCV](https://opencv.org/) - Computer Vision library used
* [MoveIt](https://moveit.ros.org/) - Motion planning framework used
* [MATLAB](https://www.mathworks.com/products/matlab.html) - Desktop environment used

## References

* [__`GitHub: lihuang3/ur5_ROS-Gazebo`__](https://github.com/lihuang3/ur5_ROS-Gazebo.git) Implementation of UR5 pick and place in ROS-Gazebo with a USB cam and vacuum grippers.
* [__`GitHub: murtazabasu/Coordinating-Two-UR5-Robots-for-a-Pick-and-Place-Task`__](https://github.com/murtazabasu/Coordinating-Two-UR5-Robots-for-a-Pick-and-Place-Task/blob/master/README.md) Coordinating Two UR5 Robots for a pick and place task
* [__`GitHub: JenniferBuehler/grasp-execution-pkgs`__](https://github.com/JenniferBuehler/grasp-execution-pkgs) grasp-execution-pkgs

## Authors

* **Peter Bassett**
* **Edward Thomson**
* **Liam Pemberton**
* **Andrew Simpson**
* **Michael Irwin**
* **Yang Chen**
* **Matthew Lim**



