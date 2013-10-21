#include "ParrotExe.hpp"
#include "ros/ros.h"
#include "quadDubins3D.h"
#include "ticpp.h"
#include "QuadCfg.h"

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
    cerr << "Error: " << error.m_details << endl;
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
  double t_limit= 1.;
  bool if_dubin_reach= false;
  
  std::cout<<"t_limit= "<<t_limit <<std::endl;
  ros::init(argc,argv,"dubin_test");
  ParrotExe parrot_exe;
  ros::Duration(1.0).sleep();
  //while 
  while(ros::ok() )
  {
    idx_uav_state= GetUavStateIdx();
    
    if( (idx_uav_state==3||idx_uav_state==7||idx_uav_state==4)
      && !if_dubin_reach )
    {
      int result= parrot_exe.DubinCommand(db_seg,t_limit);
      if(result==1) 
	parrot_exe.sendLand();
      if(result==1||result==2||result==0) 
	if_dubin_reach= true;
    }//if ends
  }//while ends

  
}//main ends
