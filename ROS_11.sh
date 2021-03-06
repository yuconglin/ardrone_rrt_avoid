#setup
. setup.sh ROS_10
#. setup.sh ROS_11
#don't forget to change ip in ardrone
NUMID_DRONE=1
#NETWORK_ROSCORE=ROS_11
DRONE_IP=192.168.1.1
LAPTOP_IDX=1
#{
#echo ./set_IP.sh $DRONE_IP
#echo exit
#} | telnet 192.168.1.1

gnome-terminal  \
	--tab --title "ardrone_driver"	--command "bash -c \"
		roslaunch ./launch/ardrone_driver.launch --wait drone_id_namespace:=drone$NUMID_DRONE drone_ip:=$DRONE_IP;
		exec bash\""  \
	--tab --title "joystick"  --command "bash -c \"
		roslaunch ./launch/joy_node.launch --wait drone_id_namespace:=drone$NUMID_DRONE;
		exec bash\""  \
	--tab --title "exe_intruder" --command "bash -c \"
		roslaunch ./launch/exe_intruder.launch --wait drone_id_namespace:=drone$NUMID_DRONE idx:=$LAPTOP_IDX>intruder_rec.txt;
		exec bash\""  \
