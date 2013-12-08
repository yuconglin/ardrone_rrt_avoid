#pragma once
//ros related
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int16.h"
//ardrone msg
#include "ardrone_rrt_avoid/DubinPath_msg.h"
#include "ardrone_rrt_avoid/ArdroneState_msg.h"
//ardrone state
#include "UavState/ArdroneState.h"


namespace Ardrone_rrt_avoid {
  class YlClRRT;
  class ParrotPlan{
    
     public:
       //constructor
       ParrotPlan(YlClRRT* _rrt_pt,char* file_log="virtual_replan_rec.txt");
       //callback functions
       void receiveCb(const std_msgs::Bool::ConstPtr& msg);
       void recNewCb(const std_msgs::Bool::ConstPtr& msg);
       void reachCb(const std_msgs::Int16::ConstPtr& msg);
       void stateCb(const ardrone_rrt_avoid::ArdroneState_msg::ConstPtr& msg);
       //set 
       inline void SetTimeLimit(const double _t_limit){t_limit= _t_limit;}
       inline void SetTOffset(const double _t_offset){t_offset=_t_offset;}
       //working part
       int working();
       int PathPlanning();
     private:
       //for planned path log
       std::ofstream log_path;
       //the planner
       YlClRRT* rrt_pt;
       //enumerations for different cases:
       enum possible_cases{PATH_READY,TREE_EXPAND,PATH_CHECK,WAIT_STATE,PATH_RECHECK,ARRIVED};
       //time
       double t_limit;
       double t_offset;
       //current state
       user_types::ArdroneState st_current;
       //flags
       bool if_receive;
       bool if_new_rec;
       int if_reach;
       bool if_state;
       //ros NodeHandle
       ros::NodeHandle nh;
       //publishers
       ros::Publisher pub_path;
       ros::Publisher pub_if_new;
       //subscribers
       ros::Subscriber sub_receive;
       ros::Subscriber sub_if_new_rec;
       ros::Subscriber sub_reach;
       ros::Subscriber sub_state;	
  };//class ends

};//namespace ends
