Udacity Robotics Nanodegree
------------------

Project 3: Where-Am-I
------------------
TODO ADD DIFFERENT PICTURE
![Robot and ball picture](overhead_shot.png)
### Overview
This project will drive a robot to chase a white ball.

### Project Setup

Navigate to the home directory in a terminal.  
Issue the following commands to create a catkin workspace:
```
mkdir -p catkin_ws/src
cd catkin_ws/src
catkin_init_workspace
```

Next, download the github repository:

```
git clone https://github.com/Wubuntu88/UdacityRobotics-Where-am-I.git
cp -R UdacityRobotics-Where-am-I/udacity_robot udacity_robot
cp -R UdacityRobotics-Where-am-I/my_robot my_robot
```

Build the catkin workspace:
```
cd ..
catkin_make
```

You must start two programs to run the system.  This is done in two different terminals.
1) The Gazebo World (RViz will also start)
```
source devel/setup.bash
roslaunch my_robot world.launch
```
2) The AMCL node
```
source devel/setup.bash
roslaunch udacity_robot amcl.launch
```

### Project Structure
Here is the project structure as listed in the project requirements:
```
.UdacityRobotics-myrobot                         # Go Chase It Project
├── my_robot                       # my_robot package                   
│   ├── launch                     # launch folder for launch files   
│   │   ├── robot_description.launch
│   │   ├── world.launch
│   ├── meshes                     # meshes folder for sensors
│   │   ├── hokuyo.dae
│   ├── urdf                       # urdf folder for xarco files
│   │   ├── my_robot.gazebo
│   │   ├── my_robot.xacro
│   ├── world                      # world folder for world files
│   │   ├── my_world
│   ├── CMakeLists.txt             # compiler instructions
│   ├── package.xml                # package info
├── ball_chaser                    # ball_chaser package                   
│   ├── launch                     # launch folder for launch files   
│   │   ├── ball_chaser.launch
│   ├── src                        # source folder for C++ scripts
│   │   ├── drive_bot.cpp
│   │   ├── process_images.cpp
│   ├── srv                        # service folder for ROS services
│   │   ├── DriveToTarget.srv
│   ├── CMakeLists.txt             # compiler instructions
│   ├── package.xml                # package info                  
└──                      
```