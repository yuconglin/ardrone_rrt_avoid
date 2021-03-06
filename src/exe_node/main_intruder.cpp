#include "ParrotExe.hpp"
#include "OthersMonitor.hpp"
#include "ros/ros.h"
#include "controller/midlevelCnt/Controller_MidLevelCnt.h"
#include "systemtime.h"
//other
#include <cfloat>
#include "armadillo"

using namespace Ardrone_rrt_avoid;

int main(int argc, char** argv)
{
   //get the system time
   std::string str_time;
   utils::getSystemTime(str_time);

   //overhead
   int laptop_idx= atoi(argv[1]);
   //file for navdata
   char file_nav[256],*xmlfile;
   if(laptop_idx== 0)
   {
     sprintf( file_nav, "/home/yucong/ros_workspace/ardrone_rrt_avoid/data/%s:%s.txt",str_time.c_str(),"other");
     xmlfile= "/home/yucong/.ros/param.xml";
   }
   else if(laptop_idx==1)
   {
   sprintf( file_nav, "/home/uav/yucong_ros_workspace/sandbox/ardrone_rrt_avoid/data/%s:%s.txt",str_time.c_str(),"other");
   xmlfile= "/home/uav/yucong_ros_workspace/sandbox/ardrone_rrt_avoid/param.xml";
   }
   else {std::cout<<"idx wrong,should be 0 or 1"<<std::endl;}
    
   int idx_uav_state = -1;
   int pre_uav_state = -1;
   double t_limit= 1e8;
   bool if_joy= false;
   ros::init(argc,argv,"the_other_ardrone");
   //the controller
   Controller_MidLevelCnt controlMid;
      
   //ParrotExe initialization
   ParrotExe parrot_exe(controlMid,file_nav,xmlfile);
   OthersMonitor monitor(1,1);
   //Set initial position
   double e= 0.6096; 
   //double x0=9*e,y0=0*e,z0=0.7,the0=M_PI;
   double x0= 10.,y0=0,z0=0.8,the0=M_PI;
   double x1=e,y1=0,z1=0.7,the1=M_PI;
   QuadCfg start(x0+5,y0,z0,the0);
   QuadCfg end(x1,y1,z1,the1);
   //parrot_exe.SetInitXY(e,0);
   //stop for settle down
   ros::Duration(1.0).sleep();
   //flat trim and take off
   parrot_exe.sendFlattrim();

   bool if_start= false;
   //the start config when switch from takeoff to hover
   QuadCfg cfg_start;
   int reach= -1;
   ros::Time t_start= ros::Time::now();
   arma::vec::fixed<3> u;
   u<<0.5<<0.<<0.;
   double dt= 0.1;

   //while
   ros::Rate r(10);
   while(ros::ok() )
   {
     idx_uav_state= parrot_exe.GetUavStateIdx();
     std::cout<<"uav_state: "<< idx_uav_state<< std::endl;
     if_joy= parrot_exe.GetIfJoy();
     parrot_exe.PubIfStable();
     //take off coordination
     //if(!parrot_exe.GetIfOff() )
     if(!parrot_exe.GetIfOff() && monitor.ifSomeTakeOff(0) )
     {//take off when it knows A takes off
       std::cout<<"take off"<< std::endl;
       parrot_exe.sendTakeoff();
     }

     if(idx_uav_state!=2 && idx_uav_state!=-1)
       parrot_exe.SetIfOff(true);

     //waiting until stable
     if(!if_start)
     {
       //find the moment takeoff-->hover
       if(pre_uav_state!= 4 && idx_uav_state==4)
       {//set the start 
	  parrot_exe.SetIfStable(true);
	  //get current absolute time
	  utils::getSystemTime(str_time);
	  std::cout<<"str_time: "<< str_time<< std::endl;
        }
        if(parrot_exe.GetIfStable() && monitor.ifOthersStable() )
	//if(parrot_exe.GetIfStable() )
        {
	  parrot_exe.GetCurrentCfg(cfg_start);
	  parrot_exe.SetStartTime(ros::Time::now() );
	  //set YawInit
	  double yaw_init= cfg_start.theta- M_PI;
	  parrot_exe.SetYawInit(yaw_init);
	  //set x_est,y_est
	  double x_init_frame= cfg_start.x*cos(yaw_init)-cfg_start.y*sin(yaw_init)+x0;
	  double y_init_frame= cfg_start.x*sin(yaw_init)+cfg_start.y*cos(yaw_init)+y0;
	  //set some
	  parrot_exe.SetXEst(x_init_frame);
	  parrot_exe.SetYEst(y_init_frame);
	  parrot_exe.SetPreXEst(x_init_frame);
	  parrot_exe.SetPreYEst(y_init_frame);
	  parrot_exe.SetPreZ(cfg_start.z);
	   
	  std::cout<<"B x_init: "<<x_init_frame<<" y_init: "<<y_init_frame<<" z_init: "<<cfg_start.z<<" the_init: "<<cfg_start.theta*180/M_PI<< std::endl;

	  parrot_exe.ControllerReset();
	  ros::Duration(1.0).sleep();
	  if_start= true;
	  parrot_exe.SetInitTimeNow();
          //get current absolute time
	  utils::getSystemTime(str_time);
	  std::cout<<"str_time: "<< str_time<< std::endl;
	}

     }//!if_start ends
     else
     {  
        if(reach!=2 && !if_joy)
	{
           reach= parrot_exe.LineCommand(start,end,t_limit);
	}
	if(reach==2) parrot_exe.sendStop();
         
        if(reach==2&&idx_uav_state==4) parrot_exe.sendLand();
        
	//publish its state
	parrot_exe.PubQuadState();
	parrot_exe.PublishFlags();
     }//else(!if_start) ends
     ros::spinOnce();
     r.sleep();
   }//while ends
   return 0;
}//main ends
