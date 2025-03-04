#!/bin/bash
sleep 1s;

# gnome-terminal --tab --title="ROSCORE" --command "bash -c \"source ~/.bashrc; killall gzclient && killall gzserver; roscore; exec bash\"";
# sleep 1s;

gnome-terminal --tab --title="Rviz" --command "bash -c \"source ~/.bashrc; roslaunch fabric_simulator rviz.launch; exec bash\"";
sleep 1s;

gnome-terminal --tab --title="Simulator" --command "bash -c \"source ~/.bashrc; roslaunch fabric_simulator fabric_simulator.launch; exec bash\"";
sleep 2s;

gnome-terminal --tab --title="Test GUI" --command "bash -c \"source ~/.bashrc; rosrun fabric_simulator test_gui.py _mode:="simulation_test" _dual_spacenav_twist:=False; exec bash\"";
sleep 1s;

# Use it to publish spacenav_twist msg
gnome-terminal --tab --title="Spacenav" --command "bash -c \"source ~/.bashrc; roslaunch spacenav_node classic.launch; exec bash\"";
# sleep 1s;

gnome-terminal --tab --title="RQT EZ Pub" --command "bash -c \"source ~/.bashrc; rosrun rqt_ez_publisher rqt_ez_publisher; exec bash\"";
sleep 1s;




