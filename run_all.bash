#!/bin/bash

echo "Run some stuff. Make your life easier!"

cd ~/ && source devel/setup.bash && gnome-terminal -- bash -c "roslaunch mavros px4.launch fcu_url:="udp://:14540@192.168.1.36:14557"" &

sleep 15 &&


cd ~/ && source devel/setup.bash && gnome-terminal -- bash -c "roslaunch ground_control ground_control.launch" &

sleep 10 &&


cd ~/ && source devel/setup.bash && gnome-terminal -- bash -c "roslaunch dynamic_tutorials server.launch" &

sleep 10 &&


cd ~/ && source devel/setup.bash && gnome-terminal -- bash -c "rosrun rqt_gui rqt_gui -s reconfigure" &

sleep 5 &&

cd ~/ && source devel/setup.bash && gnome-terminal -- bash -c "echo "Write the last command, please, by yourself""



