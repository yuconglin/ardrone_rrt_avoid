gnome-terminal \
       	--tab --title "roscore" --command "bash -c \"
       			                roscore; 
                                        exec bash\""  \
       --tab --title "ardrone_driver" --command "bash -c \"
                                        env sleep 1s ;
                                        rosrun ardrone_autonomy ardrone_driver _realtime_navdata:=True _navdata_demo:=1; 
                                        exec bash\""  \
	--tab --title "joystick" --command "bash -c \"
                                        env sleep 3s ;
                                        rosrun joy joy_node;
                                        exec bash\""  \
	--tab --title "planner"	 --command "bash -c \"
					env sleep 3s;
					roslaunch ardrone_rrt_avoid parrot_plan.launch >rec_plan.txt; 
					exec bash\""  \
	--tab --title "exe_path" --command "bash -c \"
                                        env sleep 3s ;
                                        rosrun ardrone_rrt_avoid exe_path >record.txt >rec_exe.txt;
                                        exec bash\""  \
