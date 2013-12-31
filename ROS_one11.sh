. setup.sh ROS_11
NUMID_DRONE=0
NETWORK_ROSCORE=ROS_11
LAPTOP_IDX=1
DRONE_IP=192.168.1.1

gnome-terminal  \
--tab --title "ardrone_driver"	--command "bash -c \"
	roslaunch ./launch/ardrone_driver.launch --wait drone_id_namespace:=drone$NUMID_DRONE drone_ip:=$DRONE_IP;
	exec bash\""  \
--tab --title "joystick"  --command "bash -c \"
	roslaunch ./launch/joy_node.launch --wait drone_id_namespace:=drone$NUMID_DRONE;
	exec bash\""  \
--tab --title "planner"  --command "bash -c \"
	roslaunch ./launch/parrot_plan.launch --wait drone_id_namespace:=drone$NUMID_DRONE idx:=$LAPTOP_IDX>host_plan.txt;
	exec bash\""  \
--tab --title "exe_host" --command "bash -c \"
	roslaunch ./launch/exe_host.launch --wait drone_id_namespace:=drone$NUMID_DRONE idx:=$LAPTOP_IDX>host_rec.txt;
	exec bash\""  \
