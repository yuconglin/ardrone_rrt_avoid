#pragma once
#include "ObsUpdater.hpp"
#include "ros/ros.h"
//#include "std_msgs/Bool.h"
#include "UavState/ArdroneState.h"

namespace user_types{
  class ObsUpdaterReal:public ObsUpdater{
     public:
        ObsUpdaterReal(int one,int total);
        void SeeObsUpdate();
        void SetObsUpdateFalse();
	void UpdateObs(std::vector<obstacle3D>& obs3ds); 
     private:
	int one,total;
        std::vector<user_types::ArdroneState> state_obs;
        void state_obCb(const ardrone_rrt_avoid::ArdroneState_msg::ConstPtr& msg,int _idx);

  };//class ends

};//namespace ends
