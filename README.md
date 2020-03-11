# Drone control project

To use this project, open a terminal and do the following:

```bash
$ cd ~/catkin_workspace/src
$ git clone git@github.com:Terminateit/Drones_Control.git
$ cd ..
$ catkin_make
$ source devel/setup.bash
```
The preliminary commands:
```bash
roslaunch drone_sim forest_sim.launch 
```

```bash
roslaunch ground_control ground_control.launch
```

You can run the controller using the following command:

```bash
$ roslaunch controller controller.launch
```
