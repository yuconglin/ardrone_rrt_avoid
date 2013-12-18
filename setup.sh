#copied from CVG Jesus. Credit for CVG.
NETWORK_ROSCORE=$1
# http://stackoverflow.com/questions/6482377/bash-shell-script-check-input-argument
if [ -z $NETWORK_ROSCORE ] # Check if NETWORK_ROSCORE is NULL
  then
    echo "Argument 1 is empty, setting roscore to localhost..."
    export ROS_MASTER_URI=http://localhost:11311
  else
    echo "setting roscore to argument 1"
    export ROS_MASTER_URI=http://$NETWORK_ROSCORE:11311
fi
