#include "ParrotExe.hpp"
#include "ros/ros.h"
#include "controller/midlevelCnt/Controller_MidLevelCnt.h"
#include "systemtime.h"
//other
#include <cfloat>
using namespace Ardrone_rrt_avoid;

int main(int argc, char** argv)
{ //overhead
  int idx_uav_state = -1;
  int pre_uav_state = -1;
  double t_limit= 1e8;
  //bool if_reach= false;
  bool if_joy= false;
  //ros started
  std::cout<<"t_limit= "<<t_limit <<std::endl;
  ros::init(argc,argv,"dubin_test");
  
  //the controller
  Controller_MidLevelCnt controlMid;
  //get the system time
  std::string str_time;
  utils::getSystemTime(str_time);
  
  //file for navdata
  char file_nav[256];
  sprintf( file_nav, "data/%s:%s.txt",str_time.c_str(),"path");

  //ParrotExe initialization
  ParrotExe parrot_exe(controlMid,file_nav);
  ros::Duration(1.0).sleep();
  //flat trim and take off
  parrot_exe.sendFlattrim();
  parrot_exe.sendTakeoff();
 
  bool if_start= false;
  //the start config when switch from takeoff to hover
  QuadCfg cfg_start;

  //double YAW_STANDARD= 100;//wait to measure
  //while
  while(ros::ok() )
  {
    idx_uav_state= parrot_exe.GetUavStateIdx();
    if_joy= parrot_exe.GetIfJoy();
    
    if(!if_start)
    {
      //find the moment takeoff-->hover
      if(pre_uav_state== 6 && idx_uav_state==4)
      {//set the start 
        parrot_exe.GetCurrentCfg(cfg_start);
	//parrot_exe.SetStartTime(ros::Time::now() );
	//set YawInit
	double yaw_init= cfg_start.theta;
	parrot_exe.SetYawInit(yaw_init);
	//set x_est,y_est
	double x_init_frame= cfg_start.x*cos(yaw_init)-cfg_start.y*sin(yaw_init);
	double y_init_frame= cfg_start.x*sin(yaw_init)+cfg_start.y*cos(yaw_init);
	 //set some
	 parrot_exe.SetXEst(x_init_frame);
	 parrot_exe.SetYEst(y_init_frame);
	 parrot_exe.SetPreXEst(x_init_frame);
	 parrot_exe.SetPreYEst(y_init_frame);
	 parrot_exe.SetPreZ(cfg_start.z);
	 
	 std::cout<<"x_init: "<<x_init_frame<<" y_init: "<<y_init_frame<<" z_init: "<<cfg_start.z<<" the_init: "<<cfg_start.theta*180/M_PI<< std::endl;
	 //controller reset
	 parrot_exe.ControllerReset();
         if_start= true;  
      }
    }//if(!if_start) ends
    else
    { 
      //get path msg and execute it in a fixed time
      if(  parrot_exe.GetIfRec() 
        && parrot_exe.GetIfReach()!=2
        //&& parrot_exe.GetIfNewPath()
	&& !if_joy
        )
      //if( parrot_exe.GetIfReach()!=2 && !if_joy)
      {
        //std::cout<<"execute one time"<<std::endl;
        parrot_exe.PathCommand(t_limit);
      }
      //publish its state
      parrot_exe.PubQuadState();
      parrot_exe.PublishFlags();
    }//else (!if_start) ends

    pre_uav_state= idx_uav_state;
    //let it lands
    if( parrot_exe.GetIfReach()==2 && idx_uav_state==4)
    //if(parrot_exe.GetIfReach()==2)
      parrot_exe.sendLand();

    ros::spinOnce(); 
  }//while ends
  return 0;
}//main ends
