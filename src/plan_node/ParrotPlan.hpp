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
//obstacle updater
#include "ObsUpdater/ObsUpdater.hpp"

namespace Ardrone_rrt_avoid {
  class YlClRRT;
  class ParrotPlan{
    
     public:
       //constructor
       ParrotPlan(YlClRRT* _rrt_pt,char* file_prefix="data/file");
       ~ParrotPlan();

       inline void SetTimeLimit(const double _t_limit){t_limit= _t_limit;}
       inline void SetTOffset(const double _t_offset){t_offset=_t_offset;}
       inline void SetObsUpdater(user_types::ObsUpdater* _updater_pt){this->updater_pt= _updater_pt;}
       //working part
       int working();
       int PathPlanning();
     
     private:
       //for planned path log
       //std::ofstream log_path;
       const char* file_pre;
       //the planner
       YlClRRT* rrt_pt;
       //enumerations for different cases:
       enum possible_cases{PATH_READY,TREE_EXPAND,PATH_CHECK,WAIT_STATE,PATH_RECHECK,ARRIVED};
       //time
       double t_limit;
       double t_offset;
       //current state
       user_types::ArdroneState st_current;
       //obstacle state
       user_types::ObsUpdater* updater_pt;     
       bool SeeObsUpdate();
       void UpdateObs(double t);
       
       //callback functions
       void receiveCb(const std_msgs::Bool::ConstPtr& msg);
       void recNewCb(const std_msgs::Bool::ConstPtr& msg);
       void reachCb(const std_msgs::Int16::ConstPtr& msg);
       void stateCb(const ardrone_rrt_avoid::ArdroneState_msg::ConstPtr& msg);
       //void stateCb1(const ardrone_rrt_avoid::ArdroneState_msg::ConstPtr& msg);
       //set 

       //flags
       bool if_receive;
       bool if_new_rec;
       int if_reach;
       bool if_state;
       //bool if_obs0;
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
       //ros::Subscriber sub_stateB;
  };//class ends

};//namespace ends
