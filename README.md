# Limo robot Simulation Project

## Table of Content
- [System Requirements](#system-requirements)
- [Setup](#setup)
- [Full Mission](#full-mission)
    - [Simulation](#simulation)
    - [Real Robot](#real-robot)
- [Tutorials](#follow-our-tutorials)

## System Requirements
&nbsp;&nbsp;˗&nbsp; RAM: `> 12 GB`  
&nbsp;&nbsp;˗&nbsp; CPU: `4 cores`  
&nbsp;&nbsp;˗&nbsp; GPU: `Yes`  
&nbsp;&nbsp;-&nbsp; OS: `Ubuntu 20.04 (focal)`  
&nbsp;&nbsp;-&nbsp; ROS: `noetic`  
&nbsp;&nbsp;-&nbsp; Gazebo: `Classic 11.15.1 (EOL)`  
&nbsp;&nbsp;-&nbsp; C++: `C++14`  
&nbsp;&nbsp;-&nbsp; Compiler: `g++ 9.4.0`  
&nbsp;&nbsp;-&nbsp; Build System: `catkin`  

**🔥 Breaking**

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;You can run our project on any OS, using **Docker**.  
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Refer to our [Docker Installation Guide](docker/README.md)   

## Setup

  - Install the source code
    ```bash
    git clone --recursive https://github.com/Hesham-HA/Limo-robot.git
    ```
  - Install system dependencies  

    > Hint: you won't need to do this step if you build from Docker
    <details>
      <summary>
        installation script
      </summary>
      
      ```bash
      apt-get update && apt-get install -y \
      python3-catkin-tools \
      ros-noetic-ros-control \
      ros-noetic-ros-controllers \
      ros-noetic-teleop-twist-keyboard \
      ros-noetic-moveit-simple-controller-manager \
      ros-noetic-moveit-fake-controller-manager \
      ros-noetic-moveit-setup-assistant \
      ros-noetic-moveit-planners \
      ros-noetic-moveit-ros-control-interface \
      ros-noetic-robot-pose-ekf \
      ros-noetic-gmapping \
      ros-noetic-map-server \
      ros-noetic-global-planner \
      ros-noetic-dwa-local-planner \
      ros-noetic-amcl \
      ros-noetic-move-base \
      ros-noetic-explore-lite \
      ros-noetic-rtabmap \
      ros-noetic-rtabmap-ros \
      ros-noetic-find-object-2d \
      ros-noetic-moveit-commander \
      libfmt-dev
      ```
    </details>
  - Build the code using Catkin
    ```bash
    cd limo && catkin build
    ```
  - Source code installation
    ```bash
    source devel/setup.bash
    ```
  - Start running our launch files
    `$roslaunch pick_place ${LAUNCH_FILE_NAME}.launch`

## Full Mission

### Simulation
**⚙️ Start the simulation**
```bash
roslaunch pick_place gazebo.launch
```

1. Detect object, pick it then lift it

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<img src="/screenshots/pick_part.gif" width="60%"/>

2. Detect table and Place the object

3. Search through the map

**⚙️ You can run the full mission yourself**
> ```bash
> roslaunch pick_place mission.launch
> ```
> and in another terminal call the mission service
> ```bash
> rosservice call /run/mission
> ```

### Real Robot

## Follow our tutorials  
  [Guidlines to run: SLAM, navigation, object search, pick-and-place](limo/src/pick_place/README.md)
