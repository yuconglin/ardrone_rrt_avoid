#include "ParrotExe.hpp"
#include "ros/ros.h"
#include "controller/midlevelCnt/Controller_MidLevelCnt.h"
#include "systemtime.h"
#include "armadillo"
#include "QuadCfg.h"

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
 
   double cx= 1.;
   double cy= 0.;
   double cz= 0.;
   double wz= 0.;
   arma::vec::fixed<3> u_c;
   u_c<< cx<< cy<< cz;

   //file for navdata, and dubin log
   char file_nav[256];
   sprintf( file_nav, "data/%s:%.1f:%.1f:%.1f:%.1f:%s.txt",str_time.c_str(),"u");
   //duration
   double dur= 1.0,dt= 0.1;
   //ParrotExe initialization
   ParrotExe parrot_exe(controlMid,file_nav);
   ros::Duration(1.0).sleep();
   //controller reset
   parrot_exe.ControllerReset();

   //takeoff
   parrot_exe.sendTakeoff();
     //flag to show if it is ok to start
   bool if_start= false;
   QuadCfg cfg_start;
   
   //while
   while(ros::ok())
   {
     idx_uav_state= parrot_exe.GetUavStateIdx();
     if_joy= parrot_exe.GetIfJoy();
     //to fix the start moment: takeoff---->hover
     if(pre_uav_state== 6 && idx_uav_state== 4 )
     {
       parrot_exe.GetCurrentCfg(cfg_start);
       //set YawInit
       parrot_exe.SetYawInit( cfg_start.theta );
       //set x_est,y_est
       double x_init_frame= cfg_start.x*cos(cfg_start.theta)-cfg_start.y*sin(cfg_start.theta);
       double y_init_frame= cfg_start.x*sin(cfg_start.theta)+cfg_start.y*cos(cfg_start.theta);
       //set some
       parrot_exe.SetXEst(x_init_frame);
       parrot_exe.SetYEst(y_init_frame);

       if_start= true;
     }//if ends
     
     //sending command
     if(if_start) 
     {
       parrot_exe.StepCommand(u_c,dt);
       dur-= dt;
       if(dur<= 0) if_reach= true;
     }

     if(if_reach)
     {
       parrot_exe.sendStop();
       ros::Duration(0.5).sleep();
     }
     
     pre_uav_state= idx_uav_state;
     //let it lands
     if(if_reach && idx_uav_state==4)
       parrot_exe.sendLand();
     //don't forget to spin once
     ros::spinOnce();

   }//while ends

}//main ends
