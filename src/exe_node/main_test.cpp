#include "ParrotExe.hpp"
#include "ros/ros.h"
#include "controller/midlevelCnt/Controller_MidLevelCnt.h"
#include "systemtime.h"

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
   bool if_reach= false;
   bool if_joy= false;
  
   //file for navdata, and dubin log
   char file_nav[256];
   sprintf( file_nav, "data/%s:%s.txt",str_time.c_str(),"test");
   
   //ParrotExe initialization
   ParrotExe parrot_exe(controlMid,file_nav);
   ros::Duration(1.0).sleep();
   //controller reset
   parrot_exe.ControllerReset();

   //takeoff
   parrot_exe.sendTakeoff();
   
   double cx= 1.;
   double cy= 0.;
   double cz= 0.;
   double wz= 0.;
   //while
   while(ros::ok())
   {
     idx_uav_state= parrot_exe.GetUavStateIdx();
     if_joy= parrot_exe.GetIfJoy();
     //to fix the start moment: takeoff---->hover
     if(pre_uav_state== 6 && idx_uav_state== 4 )
     {
        parrot_exe.SendCommand(cx,cy,cz,wz);
	ros::Duration(1.0).sleep();
        if_reach= true; 
     }//if ends

     if(if_reach) 
       parrot_exe.sendStop();

     pre_uav_state= idx_uav_state;
     //let it lands
     if(if_reach && idx_uav_state==4)
       parrot_exe.sendLand();
     //don't forget to spin once
     ros::spinOnce();

   }//while ends

}//main ends
