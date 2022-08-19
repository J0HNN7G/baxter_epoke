# baxter_epoke
Learning Embodied Intuitive Physics with Baxter Robot in simulation.

## Experiment

```scripts/main.py``` is the code central to experimental set up.

## ROS/Gazebo Installation

1. Source ROS distor
```
source /opt/ros/<kinetic/noetic>/setup.bash
```

2. Create workspace
```
mkdir -p ~/baxter_ws/src
```
3. Download packages
```
cd ~/baxter_ws/src
wstool init .
wstool merge https://raw.githubusercontent.com/J0HNN7G/baxter_epoke/main/<kinetic/noetic>.rosinstall
wstool update
```

4. Build ```baxter_epoke``` plugins
```
cd cd ~/baxter_ws/src/baxter_epoke/plugins
mkdir build
cd build
cmake ../
make
```

5. Build workspace
```
cd ~/baxter_ws/
catkin build
```


## ROS/Gazebo usage

Once installed, the following should have Baxter untuck its arms and be receptive to MoveIt commands via Rospy, in a world where there is a static white table, with a static purple block on top, with a moveable green cube on top:
- Terminal 1:
```
source ~/baxter_ws/devel/setup.bash
roslaunch baxter_epoke baxter_world.launch
```

- Terminal 2:
```
source ~/baxter_ws/devel/setup.bash
roslaunch baxter_epoke moveit_init.launch
```

- Terminal 3:
```
source ~/baxter_ws/devel/setup.bash
rosrun baxter_tools enable_robot.py -e
rosrun baxter_tools tuck_arms.py -u
```
