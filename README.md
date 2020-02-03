##Drones Regulator

Download the regulator from Git

```bash
git clone <URL>
```

Compile changings (go to your catkin:

```bash
cd ~/<catkin_workspace>
```

```bash
catkin_make
```

Run all:

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

Run Regulator

```bash
rosrun regulator_drone_ha1 Service_OFFMODE.py
```
