#include "ParrotExe.hpp"
#include "ros/ros.h"
#include "controller/midlevelCnt/Controller_MidLevelCnt.h"
#include "systemtime.h"

using namespace Ardrone_rrt_avoid;

int main(int argc, char** argv)
{
   double t_limit= 1.0;
   std::cout<<"t_limit= "<< t_limit <<std::endl;
   ros::init(argc,argv,"simu_path");
   
   //the controller
   Controller_MidLevelCnt controlMid;
   //get the system time
   std::string str_time;
   utils::getSystemTime(str_time);
   
   //file for navdata
   char file_nav[256];
   sprintf( file_nav, "data/%s:%s.txt",str_time.c_str(),"sim_path");
   
   //ParrotExe initialization
   ParrotExe parrot_exe(controlMid,file_nav);
   ros::Duration(1.0).sleep();

   while(ros::ok() )
   {
      //get path msg and execute it in a fixed time
      if(  parrot_exe.GetIfRec() 
	&& parrot_exe.GetIfReach()!=2
	//&& parrot_exe.GetIfNewPath()
	)
      {
	//std::cout<<"execute one time"<<std::endl;
	parrot_exe.PathCommand(t_limit);
      }
      //publish its state
      parrot_exe.SimDataUpdate();
      parrot_exe.PubQuadState();
      parrot_exe.PublishFlags();

      if(parrot_exe.GetIfReach()==2)
        cout<<"please land"<<endl;
      ros::spinOnce();
   }//while ends
   return 0;
}//main ends
