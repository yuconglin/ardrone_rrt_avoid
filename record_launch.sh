gnome-terminal  \
	--tab --title "roscore" 	--command "bash -c \"
						roscore; 
						exec bash\""  \
	--tab --title "ardrone_driver"	--command "bash -c \"
						env sleep 1s ;
						rosrun ardrone_autonomy ardrone_driver _realtime_navdata:=True _navdata_demo:=1; 
						exec bash\""  \
	--tab --title "joystick"	--command "bash -c \"
	                                        env sleep 3s ;
						rosrun joy joy_node;
						exec bash\""  \
        --tab --title "exe_record"	--command "bash -c \"
						env sleep 3s ;
					        rosrun ardrone_rrt_avoid exe_record;
						exec bash\""  \
        --tab --title "exe_record"	--command "bash -c \"
						env sleep 3s ;
					        rosrun ardrone_rrt_avoid exe_fly;
						exec bash\""  \

