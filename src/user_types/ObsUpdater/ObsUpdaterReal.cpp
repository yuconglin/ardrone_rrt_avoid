#include "ObsUpdaterReal.hpp"
#include "ros/ros.h"
#include "UavState/ArdroneState.h"
using namespace std;
namespace user_types{
  
  void StatesToObs(vector<user_types::ArdroneState> states,vector<user_types::obstacle3D>& obs);

  ObsUpdaterReal::ObsUpdaterReal(int _one,int _total):one(_one),total(_total)
  {
    for(int i=0;i!= total+1;++i)
    {
      if(i==one) continue;
      int idx= i;
      std::ostringstream convert;
      convert<< idx;
      std::string idx_str = convert.str();
      //subscriber to other vehicles' states
      ros::Subscriber sub_st= nh.subscribe<ardrone_rrt_avoid::ArdroneState_msg>(std::string("/drone")+idx_str+"/quad_state",1,boost::bind(&ObsUpdaterReal::state_obCb, this, _1, idx) );
      state_subs.push_back(sub_st);
      if_updates.push_back(false);
      state_obs.push_back(user_types::ArdroneState() );
    }//for ends
 
  }//ObsUpdaterReal constructor ends

  void ObsUpdaterReal::state_obCb(const ardrone_rrt_avoid::ArdroneState_msg::ConstPtr& msg,int _idx){
    user_types::ArdroneState st;
    st.x= msg->x;
    st.y= msg->y;
    st.z= msg->z;
    st.yaw= msg->yaw;
    st.vx= msg->vx;
    st.vy= msg->vy;
    st.vz= msg->vz;
    st.yaw_rate= msg->yaw_rate;
    st.t= msg->t;
    //push back states
    if(_idx> one) --_idx;
    state_obs[_idx] =st; 
    if_updates[_idx]= true;
  }//state_obCb ends

  bool ObsUpdaterReal::SeeObsUpdate()
  {
    for(int i=0;i!=total;++i)
      if(if_updates[i]== false) return false;
    return true;
  }//SeeObsUpdate() ends

  void ObsUpdaterReal::SetObsUpdateFalse()
  {
    for(int i=0;i!=total;++i)
      if_updates[i]= false;
  }//SetObsUpdateFalse ends

  void StatesToObs(vector<user_types::ArdroneState> states,vector<user_types::obstacle3D>& obs)
  {
    obs.clear();
    cout<<"StatesToObs"<< endl;
    for(int i=0;i!=states.size();++i)
    {
      cout<< states[i].x<<","<<states[i].y<<","<<states[i].z<<","
          << states[i].vx<<","<<states[i].vy<<","<<states[i].vz<<","<< states[i].t << endl;
      obs.push_back(states[i].toObs3D(1.,0.5) );
    }
  }//StatesToObs ends

  void ObsUpdaterReal::UpdateObs(std::vector<obstacle3D>& obs3ds,double t)
  {
    StatesToObs(state_obs,obs3ds);
    SetObsUpdateFalse();
  }//UpdateObs ends

};//namespace ends
