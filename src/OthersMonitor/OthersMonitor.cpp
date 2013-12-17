#include "ros/ros.h"
#include "OthersMonitor.hpp"
#include "boost/bind.hpp"

namespace Ardrone_rrt_avoid{
  OthersMonitor::OthersMonitor(int _one,int _total):one(_one),total(_total)
  {
    if(one< 1)
      try{
         throw std::runtime_error ("the argument one must be at least 1");
      }
         catch (std::runtime_error &e) {
          std::cout << "Caught a runtime_error exception: "
                  << e.what () << '\n';
      }
    if(total<1||total< one)
      try{
         throw std::runtime_error ("the argument total must be at least 1 or no less than the argument one");
      }
         catch (std::runtime_error &e) {
          std::cout << "Caught a runtime_error exception: "
	       << e.what () << '\n';
      }
    //initialize
    for(int i=0;i!=total;++i)
    {
      int idx= i;
      std::ostringstream convert;
      convert<< idx;
      std::string idx_str = convert.str();
      //subscriber to if take off
      ros::Subscriber sub_off= nh.subscribe(std::string("/drone")+idx_str+"/if_take_off",boost::bind(&OthersMonitor::offCb, this, _1, idx) );
      off_subs.push_back( sub_off);
      if_offs.push_back(false);
      //subscriber to if stable
      ros::Subscriber sub_stable= nh.subscribe(std::string("/drone")+idx_str+"/if_stable",boost::bind(&OthersMonitor::stableCb, this, _1, idx) );
      stable_subs.push_back(sub_stable);
      if_stable.push_back(false);
    }//for ends

  }//OthersMonitor ends
  
  bool OthersMonitor::ifOthersTakeOff()
  {
    for(int i=0;i!=total;++i)
    {
       if(i== one) continue;
       if(if_offs[i]== false)
	 return false;
    }
    return true;
  }//ifOthersTakeOff ends

  bool OthersMonitor::ifOthersStable()
  {
    for(int i=0;i!=total;++i)
    {
       if(i==one) continue;
       if(if_stables[i]== false)
	 return false;
    }
    return true;
  }//ifOthersStable ends

  //call back functions
  void OthersMonitor::offCb(std_msgs::Bool::ConstPtr& msg, int _idx)
  {
    if_offs[_idx]= msg->data;
  }//offCb ends

  void OthersMonitor::stableCb(std_msgs::Bool::ConstPtr& msg, int _idx)
  { 
    if_stables[_idx]= msg->data;
  }

}//Ardrone_rrt_avoid ends
