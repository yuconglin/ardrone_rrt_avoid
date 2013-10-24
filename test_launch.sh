gnome-terminal  \
	--tab --title "roscore" 	--command "bash -c \"
						roscore; 
						exec bash\""  \
	--tab --title "ardrone_driver"	--command "bash -c \"
						env sleep 1s ;
						rosrun ardrone_autonomy ardrone_driver _realtime_navdata:=True _navdata_demo:=1; 
						exec bash\""  \
        --tab --title "bottom_yc_img"	--command "bash -c \"
						env sleep 5s ;
						rosrun ardrone_rrt_avoid exe_dubin 1 0;
						exec bash\""  \

