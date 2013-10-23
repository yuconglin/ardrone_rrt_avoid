#include "ParrotExe.hpp"
#include "ros/ros.h"
#include "quadDubins3D.h"
#include "ticpp.h"
#include "QuadCfg.h"
#include "controller/midlevelCnt/Controller_MidLevelCnt.h"

int main(int argc, char** argv)
{ //first to get the rho
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
  double rho= v/yaw_rate;

  //construct a dubin's path in 3D
  double x_s = 0;
  double y_s = 0;
  double z_s = 0.5;
  double the_s= 0.;
  QuadCfg start(x_s,y_s,z_s,the_s);

  double x_e= x_s+ 10;
  double y_e= y_s+ 5;
  double z_e= z_s+ 0;
  double the_e= 0.;
  QuadCfg end(x_e,y_e,z_e,the_e);
   
  quadDubins3D db_3d(start,end,rho );
  //to DubinSeg struct
  ParrotExe::DubinSeg db_seg;
  db_seg.d_dubin= db_3d;
  db_seg.cfg_stop= end;
  //main part
  double if_process_start= false;
  int idx_uav_state = -1;
  int pre_uav_state = -1;
  double t_limit= 1.;
  bool if_dubin_reach= false;
  bool if_joy= false;
  bool if_start= false;
  QuadCfg cfg_start;
  
  if( argc < 2) 
    std::runtime_error("should be one argument");

  int if_manual= atoi(argv[1]);
  std::cout<<"t_limit= "<<t_limit <<std::endl;
  ros::init(argc,argv,"dubin_test");
  //the controller
  Controller_MidLevelCnt& controlMid;

  ParrotExe parrot_exe(controlMid);
  ros::Duration(1.0).sleep();
  //take off
  if(if_manual== 0)
    parrot_exe.sendTakeoff();
  else if( if_manual==1)
    std::cout<<"manual takeoff please!"<<std::endl;
  else
    std::runtime_error("wrong input. should be 1 or 2");
  
  //while 
  while(ros::ok() )
  {
    idx_uav_state= parrot_exe.GetUavStateIdx();
    if_joy= parrot_exe.GetIfJoy();
    /**************************************************
    //to navigate along a dubin's curve
    if( (idx_uav_state==3||idx_uav_state==7||idx_uav_state==4)
      && !if_dubin_reach && !if_joy )
    {
      int result= parrot_exe.DubinCommand(db_seg,t_limit);
      if(result==1) 
	parrot_exe.sendLand();
      if(result==1||result==2||result==0) 
	if_dubin_reach= true;
    }//if ends
    ****************************************************/
    //to fix the start moment: takeoff---->hover
    if(pre_uav_state== 6 && idx_uav_state== 4)
    {
      if_start= true;
      //get the dubin providing segs
      //first in the local frame, then converted to the global reference frame
      //in the global frame, start is (0,0,z_m,0) and end is(10,5,z_m,0)
      parrot_exe.GetCurrentCfg(cfg_start);
      start= QuadCfg(cfg_start.x,cfg_start.y,cfg_start.z,cfg_start.theta);
      double x= end.x*cos(cfg_start.theta)-end.y*sin(cfg_start.theta);
      double y= end.x*sin(cfg_start.theta)+end.y*cos(cfg_start.theta);
      end= QuadCfg(x, y, cfg_start.z, end.theta+cfg_start.theta);

    }
    //navigate along a segment of a dubin's curve: straight or circle

    pre_uav_state= idx_uav_state;
  }//while ends

  
}//main ends
