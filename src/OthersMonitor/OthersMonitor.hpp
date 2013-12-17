#include "ros/ros.h"

namespace Ardrone_rrt_avoid{
  //to monitor other drones states
  class OthersMonitor{
    public:
     OthersMonitor(int _one,int _total);
     bool ifOthersTakeOff();
     bool ifOthersStable();
    private: 
     int one;
     int total;
     //ros NodeHandle
     ros::NodeHandle nh;
     //subscribers to see if other vehicles starts to take off
     std::vector<ros::Subscriber> off_subs;
     //subscribers to see if other vehicles already finished taking off
     std::vector<ros::Subscriber> stable_subs;
     //flags of take off
     std::vector<bool> if_offs;
     //flags of stable
     std::vector<bool> if_stables;
     //callback functions
     void offCb(std_msgs::Bool::ConstPtr& msg, int _idx);
     void stableCb(std_msgs::Bool::ConstPtr& msg, int _idx);
  }//class OthersMonitor ends

}//Ardrone_rrt_avoid ends