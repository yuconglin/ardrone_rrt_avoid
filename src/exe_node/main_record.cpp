#include "ParrotExe.hpp"
#include "ros/ros.h"
#include "controller/midlevelCnt/Controller_MidLevelCnt.h"
#include "systemtime.h"
#include "std_msgs/Bool.h"
#include "boost/bind.hpp"

void arriveCb(const std_msgs::Bool::ConstPtr& msg,bool& IfArrive)
{
   IfArrive= msg->data;
}

using namespace Ardrone_rrt_avoid;

int main(int argc, char** argv)
{
   ros::init(argc,argv,"fly_test");

   //the controller
   Controller_MidLevelCnt controlMid;
   //get the system time
   std::string str_time;
   utils::getSystemTime(str_time);
   
   //parameters setup
   int idx_uav_state = -1;
   int pre_uav_state = -1;
   //bool if_reach= false;
   bool if_joy= false;
   bool if_start= false;
   bool if_arrive= false;
   //msgs
   std_msgs::Bool start_msg;

   //file for navdata, and dubin log
   char file_nav[256];
   sprintf( file_nav, "data/%s:%s.txt",str_time.c_str(),"test");
   
   //ParrotExe initialization
   ParrotExe parrot_exe(controlMid,file_nav);
   ros::Duration(1.0).sleep();
   //controller reset
   parrot_exe.ControllerReset();
   //get NodeHandle
   ros::NodeHandle* nh_pt= parrot_exe.GetHandlePt();
   //publisher
   ros::Publisher pub_start =nh_pt->advertise<std_msgs::Bool>("if_start",1);
   //subscriber
   ros::Subscriber sub_arrive =nh_pt->subscribe<std_msgs::Bool>("if_arrive",1,boost::bind(arriveCb,_1,boost::ref(if_arrive) ) );
   //takeoff
   parrot_exe.sendTakeoff();

   //while
   while(ros::ok() )
   {
      idx_uav_state= parrot_exe.GetUavStateIdx();
      if_joy= parrot_exe.GetIfJoy();
      
      //time to send start signal
      if(pre_uav_state== 6 && idx_uav_state== 4 )
      {
	if_start= true; 
      }//if ends

      //publish
      start_msg.data= if_start;
      pub_start.publish(start_msg);

      pre_uav_state= idx_uav_state;
      
      if(idx_uav_state== 4 && if_arrive)
	parrot_exe.sendLand();
      
      ros::spinOnce();
   }//while ends

}

