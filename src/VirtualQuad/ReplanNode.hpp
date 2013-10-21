#pragma once
#include "QuadState.h"
#include "ros/ros.h"
#include "msg.h"
//#include "VirtualQuad.hpp"

//forward declared dependencies
class VirtualQuad;

class ReplanNode{
   public:
     //constructor
     ReplanNode(VirtualQuad& _quad,int _which_obs);
     //callback functions
     void receiveCb(const yucong_rrt_avoid::IfSend_msg::ConstPtr& msg);
     void recNewCb(const yucong_rrt_avoid::IfNewRec_msg::ConstPtr& msg);
     void reachCb(const yucong_rrt_avoid::IfReach_msg::ConstPtr& msg);
     void stateCb(const yucong_rrt_avoid::QuadState_msg::ConstPtr& msg);
     //basic utilities
     int SetNormalObs();//set normal obstacles
     //inline void SetTimeLimit( const double _t_limit){t_limit= _t_limit;}
     inline void SetTOffset(const double _t_offset){t_offset=_t_offset;}
     //the main func
     int working();

   protected:  
     VirtualQuad& quad;//just a reference
     //enumerations for different cases:
     enum possible_cases{PATH_READY,TREE_EXPAND,PATH_CHECK,WAIT_STATE,PATH_RECHECK,ARRIVED};
     //basic variables
     int which_obs;
     double t_offset= 1.0;//the initial time to sleep
     QuadState st_current;
     //QuadState st_pre;
     //flags
     bool if_receive= false;
     bool if_new_rec= false;
     int if_reach= 0;
     bool if_state= false;
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
 
};//ReplanNode ends
