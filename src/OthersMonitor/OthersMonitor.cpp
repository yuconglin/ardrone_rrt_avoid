#include "OthersMonitor.hpp"
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "boost/bind.hpp"

namespace Ardrone_rrt_avoid{
  OthersMonitor::OthersMonitor(int _one,int _total):one(_one),total(_total)
  {
    if(total<0)
      try{
         throw std::runtime_error ("the argument total must be no less than 0");
      }
         catch (std::runtime_error &e) {
          std::cout << "Caught a runtime_error exception: "
	       << e.what () << '\n';
      }
    //initialize
    
    for(int i=0; i!=total+1; ++i)
    {
      //if(i==one) continue;
      int idx= i;
      std::ostringstream convert;
      convert<< idx;
      std::string idx_str = convert.str();
      //subscriber to if take off
      ros::Subscriber sub_off= nh.subscribe<std_msgs::Bool>(std::string("/drone")+idx_str+"/if_take_off",1,boost::bind(&OthersMonitor::offCb, this, _1, idx) );
      off_subs.push_back( sub_off);
      if_offs.push_back(false);
      //subscriber to if stable
      ros::Subscriber sub_stable= nh.subscribe<std_msgs::Bool>(std::string("/drone")+idx_str+"/if_stable",1,boost::bind(&OthersMonitor::stableCb, this, _1, idx) );
      stable_subs.push_back(sub_stable);
      if_stables.push_back(false);
    }//for ends
    /*
    int idx= 1-one;
    std::ostringstream convert;
    convert<< idx;
    std::string idx_str = convert.str();
    //subscriber to if take off
    off_sub= nh.subscribe<std_msgs::Bool>(std::string("/drone")+idx_str+"/if_take_off",1,boost::bind(&OthersMonitor::offCb, this, _1, idx) );
    //subscriber to if stable
    stable_sub= nh.subscribe<std_msgs::Bool>(std::string("/drone")+idx_str+"/if_stable",1,boost::bind(&OthersMonitor::stableCb, this, _1, idx) );
   */ 
  }//OthersMonitor ends
  
  bool OthersMonitor::ifOthersTakeOff()
  {
    for(int i=0;i!=total+1;++i)
    {
       if(i== one) continue;
       if(if_offs[i]== false)
	 return false;
    }
    return true;
  }//ifOthersTakeOff ends

  bool OthersMonitor::ifOthersStable()
  {
    for(int i=0;i!=total+1;++i)
    {
       if(i==one) continue;
       if(if_stables[i]== false)
	 return false;
    }
    return true;
  }//ifOthersStable ends

  bool OthersMonitor::ifSomeTakeOff(int _idx)
  {
    if(_idx==one){
      try {
        throw std::runtime_error ("the one to inspect shouldn't be the host one");
      }
      catch (std::runtime_error &e) {
        std::cout << "Caught a runtime_error exception: "
                  << e.what () << '\n';
      }
    }
    //
    if( _idx< one) return if_offs[_idx];
    return if_offs[_idx-1];
  }//ifSomeTakeOff ends

  //call back functions
  void OthersMonitor::offCb(const std_msgs::Bool::ConstPtr& msg, int _idx)
  {
    //std::cout<<"offCb: "<< _idx<<" "<<msg->data<< std::endl;
    if_offs[_idx]= msg->data;
  }//offCb ends

  void OthersMonitor::stableCb(const std_msgs::Bool::ConstPtr& msg, int _idx)
  { 
    //std::cout<<"stableCb: "<< _idx<<" "<< msg->data<< std::endl;
    if_stables[_idx]= msg->data;
  }

}//Ardrone_rrt_avoid ends
