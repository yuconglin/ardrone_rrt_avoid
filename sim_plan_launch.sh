gnome-terminal \
        --tab --title "planner"	 --command "bash -c \"
					env sleep 3s;
					roslaunch ardrone_rrt_avoid parrot_plan.launch >rec_plan.txt; 
					exec bash\""  \
	--tab --title "sim_path" --command "bash -c \"
                                        env sleep 3s ;
                                        rosrun ardrone_rrt_avoid sim_path >rec_exe.txt;
                                        exec bash\""  \
