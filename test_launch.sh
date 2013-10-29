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
        --tab --title "bottom_yc_img"	--command "bash -c \"
						env sleep 5s ;
					        #first:0--auto takeoff,1--manual takeoff.
						#second:0--straight line,1--dubin's curve
						rosrun ardrone_rrt_avoid exe_dubin 0 0 >record.txt;
						exec bash\""  \

