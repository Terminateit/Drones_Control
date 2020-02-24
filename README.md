##Drones Regulator
by Ilia Sevostianov, Daniil Kirsanov and Oleg Rodionov

## Steps:


Download the regulator from Git

```bash
cd ~/<catkin_workspace>/src/
```

```bash
git clone <URL>
```

Compile changings (go to your catkin_ws):

```bash
cd ~/<catkin_workspace>
```

```bash
catkin_make
```

```bash
source devel/setup.bash
```



Run all:


```bash
cd ~/src/Firmware
```

```bash
cd ~/src/Firmware
```

```bash
make px4_sitl gazebo
```

```bash
roslaunch mavros px4.launch fcu_url:="udp://:14540@192.168.1.36:14557"
```

```bash
roslaunch ground_control ground_control.launch
```

Run qgroundcontrol

Run dynamic reconfigurator

```bash
roslaunch dynamic_tutorials server.launch
```

```bash
rosrun rqt_gui rqt_gui -s reconfigure
```

Run Regulator

```bash
rosrun regulator_drone_ha1 Service_OFFMODE.py
```
