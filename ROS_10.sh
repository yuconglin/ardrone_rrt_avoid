#Setup
. setup.sh
#don't forget to change ip in ardrone
#NUMID_DRONE=$1
NUMID_DRONE=0
#NETWORK_ROSCORE=$2
#NETWORK_ROSCORE=ROS_10
#DRONE_IP=$3
LAPTOP_IDX=0
DRONE_IP=192.168.1.1

#{
#echo ./set_IP_.sh $DRONE_IP
#echo exit
#} | telnet 192.168.1.1

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
