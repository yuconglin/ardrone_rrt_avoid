#include "ParrotExe.hpp"
#include "ros/ros.h"
#include "quadDubins3D.h"
#include "ticpp.h"
#include "QuadCfg.h"
#include "controller/midlevelCnt/Controller_MidLevelCnt.h"
#include "systemtime.h"

using namespace Ardrone_rrt_avoid;
int GetRho(double& rho);

int main(int argc, char** argv)
{ //first to get the rho
  double rho= 0;
  GetRho(rho);
  std::cout<<"rho= "<<rho<<std::endl;

  //construct a dubin's path in 3D
  double x_s = 0;
  double y_s = 0;
  double z_s = 0.78;
  double the_s= 0;
  QuadCfg start(x_s,y_s,z_s,the_s);

  double x_e= x_s+1.;
  double y_e= y_s+0.;
  //double x_e= x_s+ rho;
  //double y_e= y_s+ rho;
  double z_e= z_s+ 0.;
  double the_e= the_s;
  //double the_e= the_s+ M_PI/2;
  QuadCfg end(x_e,y_e,z_e,the_e);
   
  double type= (the_e > the_s ? L_SEG : R_SEG ); 
  
  //two options
  int option= atoi(argv[2]);
  //instantiate a dubin_seg structure when needed
  quadDubins3D db_3d(start,end,rho);
  //to DubinSeg struct
  ParrotExe::DubinSeg db_seg;
  db_seg.d_dubin= db_3d;
  db_seg.cfg_stop= end;
  
  //main part
  double if_process_start= false;
  int idx_uav_state = -1;
  int pre_uav_state = -1;
  double t_limit= 1e7;
  bool if_reach= false;
  bool if_joy= false;
  bool if_start= false;
  //the start config when switch from takeoff to hover
  QuadCfg cfg_start;
  
  if( argc < 3) 
    std::runtime_error("should be one argument");

  std::cout<<"t_limit= "<<t_limit <<std::endl;
  ros::init(argc,argv,"dubin_test");
  
  //the controller
  Controller_MidLevelCnt controlMid;
  //get the system time
  std::string str_time;
  utils::getSystemTime(str_time);
  
  //file for navdata, and dubin log
  char file_nav[256],file_dubin[256];
  sprintf( file_nav, "data/%s:%.1f:%.1f:%.1f:%s.txt",str_time.c_str(),x_e,y_e,z_e,"nav");
  if(option==1)
  {
    sprintf( file_dubin, "data/%s:%.1f:%.1f:%.1f:%s.txt",str_time.c_str(),x_e,y_e,z_e,"dubin");
    db_3d.OutDubins(0.1,file_dubin);
  }
  
  //ParrotExe initialization
  ParrotExe parrot_exe(controlMid,file_nav);
  std::cout<<"now init: "<<ros::Time::now().toSec()<<std::endl;
  ros::Duration(1.0).sleep();
  
  //take off
  int if_manual= atoi(argv[1]);
  if(if_manual== 0)
    parrot_exe.sendTakeoff();
  else if( if_manual==1)
    std::cout<<"manual takeoff please!"<<std::endl;
  else
    std::runtime_error("wrong input. should be 0 or 1");
  
  //while 
  while(ros::ok() )
  {
    idx_uav_state= parrot_exe.GetUavStateIdx();
    if_joy= parrot_exe.GetIfJoy();
    //to fix the start moment: takeoff---->hover
    if(pre_uav_state== 6 && idx_uav_state== 4)
    //if(pre_uav_state== -1 && idx_uav_state== 6)
    {
      //get the dubin providing segs
      //first in the local frame, then converted to the global reference frame
      //in the global frame, start is (0,0,z_m,0) and end is(10,5,z_m,0)
      parrot_exe.GetCurrentCfg(cfg_start);
      parrot_exe.SetStartTime(ros::Time::now() );
      //set YawInit
      parrot_exe.SetYawInit( cfg_start.theta );
      //set x_est,y_est
      double x_init_frame= cfg_start.x*cos(cfg_start.theta)-cfg_start.y*sin(cfg_start.theta);
      double y_init_frame= cfg_start.x*sin(cfg_start.theta)+cfg_start.y*cos(cfg_start.theta);
      //set some
      parrot_exe.SetXEst(x_init_frame);
      parrot_exe.SetYEst(y_init_frame);
      parrot_exe.SetPreXEst(x_init_frame);
      parrot_exe.SetPreYEst(y_init_frame);
      parrot_exe.SetPreZ(cfg_start.z);
      
      std::cout<<"x_init: "<<x_init_frame<<" y_init: "<<y_init_frame<<" z_init: "<<cfg_start.z<<" the_init: "<<cfg_start.theta*180/M_PI<< std::endl;
      //controller reset
      parrot_exe.ControllerReset();
    }
    
    if(option==0)
    {//navigate along a straight line or a circle curve
      if( (idx_uav_state==3||idx_uav_state==7||idx_uav_state==4)
          && !if_reach && !if_joy )
      {
	//std::cout << "fly phase"<<std::endl;
	int result= parrot_exe.LineCommand(start,end,t_limit);
	//int result= parrot_exe.CircleCommand(start,end,type,rho,t_limit);
	if(result==2)
	{ 
	  parrot_exe.sendStop();
	  std::cout<<"end reached,yeah"<<std::endl;
	}
	if(result==0)
	{
          parrot_exe.sendStop();
	  std::cout<<"time limit reached"<<std::endl;
	}
	if(result==0||result==2)
	{
	  if_reach= true;
	}
      }//if
    }
    else if(option==1)
    {//the dubin's curve
      if( (idx_uav_state==3||idx_uav_state==7||idx_uav_state==4)
      && !if_reach && !if_joy )
      {
        int result= parrot_exe.DubinCommand(db_seg,t_limit);
        //if(result==1) 
	//  parrot_exe.sendStop();
        if(result==1||result==2||result==0) 
	{
	  if_reach= true;
	  parrot_exe.sendStop();
	  if(result==0)
	    std::cout<<"time up"<<std::endl;
	  if(result==1)
	    std::cout<<"target reached"<<std::endl;
	  else
            std::cout<<"end reached"<<std::endl;
	}//if result ends
      }//if ends
    }
    else
     std::runtime_error("wrong input. should be 0 or 1");

    pre_uav_state= idx_uav_state;
    //let it lands
    if(if_reach && idx_uav_state==4)
      parrot_exe.sendLand();
    //don't forget to spin once
    ros::spinOnce();

  }//while ends
 
}//main ends

int GetRho(double& rho)
{
  double v,yaw_rate;
  try
  {
    const char* pFilename="/home/yucong/.ros/param.xml";
    ticpp::Document doc(pFilename);
    // actually load the information
    doc.LoadFile();
    //the element to start
    ticpp::Element *child= doc.FirstChildElement("quadrotor_param")->FirstChildElement();
    //an iterator
    ticpp::Iterator<ticpp::Element> iter(child);
    for( iter=iter; iter!=iter.end(); ++iter )
    {
       std::string strName;
       iter->GetValue(&strName);
       //
       if( strName== "velocity_xy" )
	 v= atof(iter->GetText().c_str() );
       if( strName== "yaw_rate")
	 yaw_rate= atof(iter->GetText().c_str() );
    }//for child ends

  }
  catch(ticpp::Exception& error)
  {
    std::cerr << "Error: " << error.m_details << std::endl;
    return 2;                 // signal error
  }
  rho= v/yaw_rate;
  return 0;
}//GetRho ends
