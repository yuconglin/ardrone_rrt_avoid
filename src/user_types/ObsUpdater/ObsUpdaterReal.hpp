#pragma once
#include "ObsUpdater.hpp"
#include "ros/ros.h"
//#include "std_msgs/Bool.h"
#include "ardrone_rrt_avoid/ArdroneState_msg.h"
#include "UavState/ArdroneState.h"

namespace user_types{
  class ObsUpdaterReal:public ObsUpdater{
     public:
        ObsUpdaterReal(int one,int total);
        bool SeeObsUpdate();
        void SetObsUpdateFalse();
	void UpdateObs(std::vector<obstacle3D>& obs3ds,double t); 
     private:
	int one,total;
	ros::NodeHandle nh;
        std::vector<ros::Subscriber> state_subs; 
        std::vector<user_types::ArdroneState> state_obs;
        void state_obCb(const ardrone_rrt_avoid::ArdroneState_msg::ConstPtr& msg,int _idx);

  };//class ends

};//namespace ends
