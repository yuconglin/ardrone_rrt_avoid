gnome-terminal  \
	--tab --title "roscore" 	--command "bash -c \"
						roscore; 
						exec bash\""  \
	--tab --title "ardrone_driver"	--command "bash -c \"
						env sleep 5s ;
						rosrun ardrone_autonomy ardrone_driver _realtime_navdata:=True _navdata_demo:=1; 
						exec bash\""  \
	--tab --title "joystick"	--command "bash -c \"
	                                        env sleep 5s ;
						rosrun joy joy_node;
						exec bash\""  \
        --tab --title "exe_record"	--command "bash -c \"
						env sleep 5s ;
					        rosrun ardrone_rrt_avoid exe_record 0;
						exec bash\""  \
        --tab --title "exe_fly"	--command "bash -c \"
						env sleep 5s ;
					        rosrun ardrone_rrt_avoid exe_fly >fly_record.txt;
						exec bash\""  \

