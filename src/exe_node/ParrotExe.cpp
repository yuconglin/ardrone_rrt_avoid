#define IF_SIM 0

#include "ParrotExe.hpp"
#include "ticpp.h"
#include "ros/ros.h"
#include "armadillo"
#include "dubins.h"
#include "quadDubins3D.h"
#include "fstream"
//controller
#include "controller/midlevelCnt/Controller_MidLevelCnt.h"
#include "controller/midlevelCnt/Controller_MidLevel_controlModes.h"
#include "Other/jesus_library.h"
//messages
#include "ardrone_rrt_avoid/DubinPath_msg.h"
#include "ardrone_rrt_avoid/DubinSeg_msg.h"
#include "ardrone_rrt_avoid/ArdroneState_msg.h"
//ros messages
#include "std_msgs/Empty.h"
#include "gazebo_msgs/GetModelState.h"
//utils
#include "QuatRPY.h"

using namespace std;
namespace Ardrone_rrt_avoid{

int ParrotExe::ParamFromXML(const char* pFilename)
{
   try
   {
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
	   this->v= atof(iter->GetText().c_str() );
         if( strName== "velocity_z" )
	   this->vz= atof(iter->GetText().c_str() );
         if( strName== "dt")
	   this->dt= atof(iter->GetText().c_str() );
         if( strName== "yaw_rate")
 	   this->yaw_rate= atof(iter->GetText().c_str() );
     }//for child ends

   }
   catch(ticpp::Exception& error)
   {
     cerr << "Error: " << error.m_details << endl;
     return 2;                 // signal error
   }
   return 0;
}//ParamFromXML ends

ParrotExe::ParrotExe(Controller_MidLevelCnt& _controlMid,char* file_nav,const char* xmlfilename):controlMid(_controlMid),log_path("ardrone_path_rec.txt"),log_nav(file_nav)
{
   cout<<"initialized"<<endl;
   //flags default
   if_receive= false;
   if_reach= 0;
   if_new_path= false;
   if_new_rec= false;
   if_joy= false;
   uav_state_idx= -1;
   lastR1Pressed= false;
   lastL1Pressed= false;
   //publishers
   //for planning related
   pub_rec = nh.advertise<std_msgs::Bool>("path_rec",1);
   pub_vel = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
   pub_reach= nh.advertise<std_msgs::Int16>("quad_reach",1);
   pub_state= nh.advertise<ardrone_rrt_avoid::ArdroneState_msg>("quad_state",1); 
   pub_stateB= nh.advertise<ardrone_rrt_avoid::ArdroneState_msg>("quad_stateB",1);
   
   pub_new_rec= nh.advertise<std_msgs::Bool>("if_new_rec",1);
   pub_state_idx= nh.advertise<std_msgs::Int16>("state_id",1);
   //for quad related
   takeoff_pub= nh.advertise<std_msgs::Empty>("ardrone/takeoff",1);
   land_pub= nh.advertise<std_msgs::Empty>("ardrone/land",1);
   emergency_pub= nh.advertise<std_msgs::Empty>("ardrone/reset",1);
   //A and B
   pub_ifoff_A= nh.advertise<std_msgs::Bool>("if_off_A",1);
   pub_ifoff_B= nh.advertise<std_msgs::Bool>("if_off_B",1);
   pub_ifstable_A= nh.advertise<std_msgs::Bool>("if_stable_A",1);
   pub_ifstable_B= nh.advertise<std_msgs::Bool>("if_stable_B",1);
   
   sub_ifoff_A= nh.subscribe("if_off_A",1,&ParrotExe::A_ifoffCb,this);
   sub_ifoff_B= nh.subscribe("if_off_B",1,&ParrotExe::B_ifoffCb,this);
   sub_ifstable_A= nh.subscribe("if_stable_A",1,&ParrotExe::A_ifstableCb,this);
   sub_ifstable_B= nh.subscribe("if_stable_B",1,&ParrotExe::B_ifstableCb,this);

   //Subscribers
   sub_path = nh.subscribe("path", 1, &ParrotExe::pathCallback,this);
   sub_if_new= nh.subscribe("if_new_path",1, &ParrotExe::newCallback,this);
   joy_sub= nh.subscribe(nh.resolveName("joy"), 1, &ParrotExe::joyCb, this);
   nav_sub= nh.subscribe("ardrone/navdata", 1, &ParrotExe::navdataCb, this);
   //for simulation
#if IF_SIM == 1
   gmscl= nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
   gmscl.waitForExistence();
   getmodelstate.request.model_name = "quadrotor";
#endif
   //services
   flattrim_srv= nh.serviceClient<std_srvs::Empty>(nh.resolveName("ardrone/flattrim"),1);

   //specify some parameters of the quad
   //if(ParamFromXML("/home/yucong/.ros/param.xml")!=0)
   //if(ParamFromXML("/home/uav/yucong_ros_workspace/sandbox/ardrone_rrt_avoid/param.xml")!=0)
   if(ParamFromXML(xmlfilename)!=0)
   {
     try {
        throw std::runtime_error ("ParamFromXML error");
     }
     catch (std::runtime_error &e) {
        std::cout << "Caught a runtime_error exception: "
                  << e.what () << '\n';
     }
   }
   //parameters
   rho= v/yaw_rate;
   speed= sqrt(v*v+vz*vz);
   //end_r= max(speed*dt,0.2);
   end_r= speed*dt;
   //controller related
   tkm1 = ros::Time::now();
   tk = ros::Time::now();
   elapsed_time = tk - tkm1;
   elapsed_time_dbl = elapsed_time.sec + ( (double) elapsed_time.nsec)/1E9;
   x_est = 0.0, y_est = 0.0, z_mea = 0.;
   x_pre = 0.0, y_pre = 0.0, z_pre = 0., yaw_pre = 0.;
   zm_pre = 0.0;
   x_ini= 0.0, y_ini= 0.0;

   vxm_est = 0.0, vym_est = 0.0, vzm_est = 0.0, yaw_est = 0.0; // meter/s
   yawci = 0.0,  vxfi = 0.0, vyfi = 0.0, dzfi=0.; // meter/s
   pitchco = 0.0, rollco = 0.0, dyawco = 0.0, dzco = 0.0;
   YawInit= 0.0;
   // For trajectory control
   controlMid.setControlMode(Controller_MidLevel_controlMode::SPEED_CONTROL);
}
/*
void ParrotExe::SetInitXY(double _x,double _y)
{
   x_ini= _x;
   y_ini= _y;
}*/

void ParrotExe::SetInitTimeNow()
{
   t_init= ros::Time::now().toSec();
}//SetInitTimeNow() ends

void ParrotExe::SetStartTimeNow()
{
   t_start= ros::Time::now();
   std::cout<<"now t_start: "<<t_start.toSec()<< std::endl;
}

void ParrotExe::A_ifoffCb(const std_msgs::Bool::ConstPtr& msg)
{
   if_off_A= msg->data; 
}//A_ifoffCb

void ParrotExe::B_ifoffCb(const std_msgs::Bool::ConstPtr& msg)
{
   if_off_B= msg->data;
}//B_ifoffCb

void ParrotExe::A_ifstableCb(const std_msgs::Bool::ConstPtr& msg)
{
   if_stable_A= msg->data;
}//A_ifstableCb

void ParrotExe::B_ifstableCb(const std_msgs::Bool::ConstPtr& msg)
{ 
   if_stable_B= msg->data; 
}//B_ifstableCb

void ParrotExe::PubIfOff(int idx)
{
   std_msgs::Bool msg;
   
   if(idx==0){
     msg.data= if_off_A;
     pub_ifoff_A.publish(msg);
   }
   else if(idx==1){
     msg.data= if_off_B;
     pub_ifoff_B.publish(msg);
   }
   else {;}
}//PubIfOff ends

void ParrotExe::PubIfStable(int idx)
{
   std_msgs::Bool msg;

   if(idx==0){
     msg.data= if_stable_A;
     pub_ifstable_A.publish(msg);
   }
   else if(idx==1){
     msg.data= if_stable_B;
     pub_ifstable_B.publish(msg);
   }
   else {;}
}//PubIfStable ends

void ParrotExe::pathCallback(const ardrone_rrt_avoid::DubinPath_msg::ConstPtr& msg)
{
   if(!if_receive)
   {
     path_msg= *msg;
     if_receive= true;
   }//end if
}//pathCallback ends

void ParrotExe::newCallback(const std_msgs::Bool::ConstPtr& msg)
{
   if_new_path= msg->data;
   if(!if_new_rec)
   {
     if_new_rec= true;
   }

   if(if_new_path && if_reach!=2) 
   {
     //std::cout<<"tatatata"<<std::endl;
     if_reach= 0;
   }
}//newCallback ends

void ParrotExe::navdataCb(const ardrone_autonomy::NavdataConstPtr navdataPtr)
{
   //position estimator:x,y ordometry; z altitude measurement but it should be kalman filetered
   //time
   tk = navdataPtr->header.stamp;
   
   elapsed_time = tk - tkm1;
   
   elapsed_time_dbl= elapsed_time.sec+ (double)elapsed_time.nsec/1E9;
   //velocity
   vxm_est = (double)navdataPtr->vx/1000.0; 
   vym_est = (double)navdataPtr->vy/1000.0;
   //vzm_est = (double)navdataPtr->vz/1000.0;
   yaw_est = (double)navdataPtr->rotZ*M_PI/180;
   yaw_est -= YawInit;

   if(yaw_est> M_PI) 
     yaw_est-= 2*M_PI;
   else if(yaw_est< -M_PI) 
     yaw_est+= 2*M_PI;
   else ;//do nothing
   
   vx_est = vxm_est*cos(yaw_est) - vym_est*sin(yaw_est); 
   vy_est = vxm_est*sin(yaw_est) + vym_est*cos(yaw_est);
   //yaw rate
   wz_est = (yaw_est-yaw_pre)/elapsed_time_dbl;
   //position
   x_est += elapsed_time_dbl * vx_est;
   y_est += elapsed_time_dbl * vy_est;
   z_mea = navdataPtr->altd/1000.;
   //vz from differention
   vzm_est =(z_mea-zm_pre)/elapsed_time_dbl;
   //which state the quad is
   uav_state_idx= navdataPtr->state;

   tkm1= tk;
   yaw_pre= yaw_est;
   zm_pre= z_mea;
   //cout<<"navdata: "<< x_est<<" "<<y_est<<" "<<z_mea<<endl;
   //log
   log_nav<<tk<<" "<<x_est<<" "<<y_est<<" "<<z_mea<<" "<<yaw_est*180/M_PI<<" "<<vx_est<<" "<<vy_est<<" "<<" "<<vzm_est<<" "<<wz_est<<" "<<uav_state_idx<<endl;

}//navdataCb ends

void ParrotExe::SimDataUpdate()
{
   double q_x,q_y,q_z,q_w,roll,pitch;
   gmscl.call(getmodelstate);
   //pose
   x_est = getmodelstate.response.pose.position.x;
   y_est = getmodelstate.response.pose.position.y;
   z_mea = getmodelstate.response.pose.position.z;
   q_x = getmodelstate.response.pose.orientation.x;
   q_y = getmodelstate.response.pose.orientation.y;
   q_z = getmodelstate.response.pose.orientation.z;
   q_w = getmodelstate.response.pose.orientation.w;
   utils::Quat2RPY(q_x,q_y,q_z,q_w,roll,pitch,yaw_est);
   //twist
   vx_est= getmodelstate.response.twist.linear.x;
   vy_est= getmodelstate.response.twist.linear.y;
   vzm_est= getmodelstate.response.twist.linear.z;
   wz_est= getmodelstate.response.twist.angular.z;
   //time
   tk= ros::Time::now();
   
}//dataUpdate ends

int ParrotExe::GetCurrentCfg(QuadCfg& cfg)
{
   cfg= QuadCfg(x_est,y_est,z_mea,yaw_est);
   return 0;
}//GetCurrentCfg ends

void ParrotExe::joyCb(const sensor_msgs::JoyConstPtr joy_msg)
{
   //cout<<"joycb joycb"<<endl; 
   if(joy_msg->axes.size() < 4) {
        ROS_WARN_ONCE("Error: Non-compatible Joystick!");
        return;
    }
    // Avoid crashes if non-ps3 joystick is being used
    short unsigned int L1 = 4;
    short unsigned int R1 = 5;
    /*previous set. European mode
    int YAW = 3;
    int GAZ = 4;
    int ROLL = 0;
    int PITCH = 1;
    */
    //current set. U.S. mode
    int YAW= 0;
    int GAZ= 1;
    int ROLL= 3;
    int PITCH= 4;

    if( joy_msg->buttons.at(2) )
	if_joy = false;

    bool justStartedControlling = false;
    if(joy_msg->axes[YAW]>0.1||joy_msg->axes[YAW]<-0.1||
       joy_msg->axes[GAZ]>0.1||joy_msg->axes[GAZ]< -0.1||
       joy_msg->axes[ROLL]>0.1||joy_msg->axes[ROLL]<-0.1||
       joy_msg->axes[PITCH]>0.1||joy_msg->axes[PITCH]<-0.1||
       joy_msg->buttons.at(R1) //|| joy_msg->buttons.at(L1)  
      )
    {
        if_joy = true;
        justStartedControlling = true;
        cout << "start control" << endl;
    }
    // are we actually controlling with the Joystick?
    if( justStartedControlling || if_joy )
    {
 	ControlCommand c;
	c.yaw = joy_msg->axes[YAW];
	c.gaz = joy_msg->axes[GAZ];
	c.roll = joy_msg->axes[ROLL];
	c.pitch = -joy_msg->axes[PITCH];

	SendControlToDrone(c);
	//cout << lastL1Pressed << " " << joy_msg->buttons.at(L1) << endl;
        if( joy_msg->buttons.at(L1) )
	{
          if( uav_state_idx == 2 ){
            cout<<"trying to take off"<<endl;
	    sendTakeoff();
	  }
	  if( uav_state_idx == 3 || uav_state_idx == 7 ){
	    SendControlToDrone( ControlCommand(0,0,0,0) );
	    sendLand();
	  }
          if( uav_state_idx == 4 )
	    sendLand();
	}//if joy_msg ends

        if(!lastR1Pressed && joy_msg->buttons.at(R1))
			sendEmergencyStop();
       }
    lastL1Pressed =joy_msg->buttons.at(L1);
    lastR1Pressed = joy_msg->buttons.at(R1);
    if( if_joy ) cout << "joyjoy" << endl;
 
}//joyCb ends

void ParrotExe::PublishFlags()
{//to publish flags 
   rec_msg.data= if_receive;
   pub_rec.publish(rec_msg);
   
   new_rec_msg.data= if_new_rec;
   pub_new_rec.publish(new_rec_msg);
   
   reach_msg.data= if_reach;
   pub_reach.publish(reach_msg);
   
   state_idx_msg.data= uav_state_idx;
   pub_state_idx.publish(state_idx_msg);
}//PublishFlags ends

void ParrotExe::PubQuadState(int idx)
{
   state_msg.x= x_est;
   state_msg.y= y_est;
   state_msg.z= z_mea;
   state_msg.yaw= yaw_est;
   state_msg.vx= vx_est;
   state_msg.vy= vy_est;
   state_msg.vz= vzm_est;
   state_msg.yaw_rate= wz_est;
   state_msg.t= tk.toSec()-t_init;
   //pub the state
   if(idx==0)
     pub_state.publish(state_msg);
   else if(idx==1)
     pub_stateB.publish(state_msg);
   else
     {;}
}//PubQuadState ends

void ParrotExe::SendControlToDrone(ControlCommand cmd)
{
   // TODO: check converstion (!)
   geometry_msgs::Twist cmdT;
   cmdT.angular.z = cmd.yaw;
   cmdT.linear.z = cmd.gaz;
   cmdT.linear.x = -cmd.pitch;
   cmdT.linear.y = cmd.roll;
   pub_vel.publish(cmdT);
}

//for test from outside
void ParrotExe::SendCommand(double cx,double cy,double cz,double cw)
{
   SendControlToDrone( ControlCommand(-cx,cy,cz,cw) );
}//SendCommand ends

void ParrotExe::sendLand()
{
   land_pub.publish(std_msgs::Empty());
   cout << "land land" << endl;
}

void ParrotExe::sendTakeoff()
{
   takeoff_pub.publish(std_msgs::Empty());
   //cout << "take off" << endl;
}

void ParrotExe::sendStop()
{
   SendControlToDrone( ControlCommand(0,0,0,0) ); 
}

void ParrotExe::sendEmergencyStop()
{//send to emergency stop
   emergency_pub.publish(std_msgs::Empty() );
}

void ParrotExe::sendFlattrim()
{
   flattrim_srv.call(flattrim_srv_srvs);
}

void ParrotExe::SetRestartDefault()
{
   if_restart_path= true;
   if_restart_dubin= true;
   if_restart_seg= true;
   idx_dubin= -1;
   idx_dubin_sub= -1;
}//SetRestartDefault ends

void ParrotExe::ControllerReset()
{
   controlMid.reset(); 
}//ControllerReset() ends

int ParrotExe::PathCommand(const double _t_limit)
{   //if no path, just stop and hover
   //cout<<"PathCommand"<< endl;
   //cout<<"path t_limit:"<< _t_limit << endl;
   if(if_reach==2)
   {
     SendControlToDrone(ControlCommand(0,0,0,0) );
     ros::Duration(dt).sleep();
     cout<<"arrived, try to land"<< endl;
     return 0;
   }

   if(if_restart_path && path_msg.dubin_path.size()==0 )
   { //command it to stop. for fixed wing, maybe other mechnism
     SendControlToDrone( ControlCommand(0,0,0,0) );
     //require a new path
     if_receive= false;
     if_new_path= false;
     if_new_rec= false; 
     //restart to default
     SetRestartDefault();
     cout<<"no path,stop"<<endl;
     return 0;
   }
   
   //first see if the current position is already at the goal
   ardrone_rrt_avoid::DubinSeg_msg dubin_last = path_msg.dubin_path.back();
   
   if( sqrt(pow(x_est-dubin_last.stop_pt.x,2)+pow(y_est-dubin_last.stop_pt.y,2)+pow(z_mea-dubin_last.stop_pt.z,2)) < end_r )
   {
     SendControlToDrone( ControlCommand(0,0,0,0) );
     if_reach= 2;
     if_receive= false;
     if_new_path= false;
     if_new_rec= false; 
     //restart to default
     SetRestartDefault();
     cout<<"got target already"<<endl;
     return 0;
   }
   
   if(if_restart_path)
   {
     if_restart_path= false;
     //time start to follow a newly received path
     t_start= ros::Time::now();
     //if none of the above happens just convert msg to vector of DubinSegs
     if(!dubin_segs.empty() ) dubin_segs.clear();
     
     for(int i=0;i!=path_msg.dubin_path.size();++i)
     {
       ardrone_rrt_avoid::DubinSeg_msg db_msg= path_msg.dubin_path[i];
       QuadCfg start(db_msg.d_dubin.start.x,db_msg.d_dubin.start.y,db_msg.d_dubin.start.z,db_msg.d_dubin.start.theta);
       QuadCfg end(db_msg.d_dubin.end.x,db_msg.d_dubin.end.y,db_msg.d_dubin.end.z,db_msg.d_dubin.end.theta);
       quadDubins3D db_3d(start,end,rho);
       //std::cout<<i<<" seg length: "<<db_3d.seg_param[0]<<" "<<db_3d.seg_param[1]<<" "<<db_3d.seg_param[2]<<std::endl;
       QuadCfg stop(db_msg.stop_pt.x,db_msg.stop_pt.y,db_msg.stop_pt.z,0);
       //std::cout<<"to msg stop: "<<stop.x<<" "<<stop.y<<" "<<stop.z<<std::endl;
       //print each of the dubin's curve
       std::cout<<i <<" start: "<< start.x<<" "<<start.y<<" "<<start.z<< std::endl;
       std::cout<<i <<" stop: "<< stop.x<<" "<< stop.y<<" "<< stop.z<< std::endl;
       //print ends
       DubinSeg db_seg;
       db_seg.d_dubin= db_3d;
       db_seg.cfg_stop= stop;
       dubin_segs.push_back(db_seg); 
     }//for int i ends
     
     //see which DubinSeg it shall follow
     int idx_sec= -1;
     if(dubin_segs.size()>1)
     {
       int idx= -1;
       double dwp[dubin_segs.size()+1];
       double len_wp[dubin_segs.size()];

       double dis_temp= 1e8;
	     
       for(int i=0;i!=dubin_segs.size();++i)
       {
	  //get the start point of each dubins curve
	  QuadCfg start= dubin_segs[i].d_dubin.cfg_start; 
	  double dis=pow(start.x-x_est,2)+pow(start.y-y_est,2)+pow(start.z-z_mea,2);
	  dwp[i]= sqrt(dis);
	  if(dwp[i]<dis_temp )
	  {
	    dis_temp= dwp[i];
	    idx= i;
	  }//if dis ends
          
	  QuadCfg stop= dubin_segs[i].cfg_stop; 
	  len_wp[i]= dubin_segs[i].d_dubin.CloseLength(stop.x,stop.y,stop.z);
	  
       }//for int i ends
       //the final stop point
       QuadCfg stop= dubin_segs.back().cfg_stop;
       double dis=pow(stop.x-x_est,2)+pow(stop.y-y_est,2)+pow(stop.z-z_mea,2);
       dis= sqrt(dis);
       dwp[dubin_segs.size()]= dis;
       if(dis<dis_temp) idx= dubin_segs.size();
	     
       if( idx!=0 && idx!=dubin_segs.size() )
       {
	 if( dwp[idx-1]/(len_wp[idx-1]+1e-8)< dwp[idx+1]/(len_wp[idx]+1e-8) )
	   idx_sec= idx-1;
	 else
	   idx_sec= idx;
       }
       else if(idx==dubin_segs.size() )
	 idx_sec= idx-1;
       else
	 idx_sec= idx;
     }//if(dubin_segs.size()>1) ends
     else
       idx_sec= 0;
     //assign idx_dubin
     idx_dubin= idx_sec;
            //then see which seg of the dubins curve it should follow
   }//if_restart_path ends
   
   else{
     
     if(idx_dubin== -1)
     {
        try {
          throw std::runtime_error ("error:idx_dubin unassigned");
        }
        catch (std::runtime_error &e) {
          std::cout << "Caught a runtime_error exception: "
                  << e.what () << '\n';
        }
     }
   }//else ends
   
   //yeah, let's follow that dubin's curve 
   //cout<<"execute one time: "<<"idx_dubin: "<< idx_dubin<< endl; 
   int result= DubinCommand(dubin_segs[idx_dubin], _t_limit);
   //reaching the end of a dubin's curve
   if(result==1 || result==2)
   {
     //cout<<"kusa kusa"<< endl;
     if(idx_dubin== dubin_segs.size()-1) 
     {//if it is the last dubin's curve
       if_reach= 2; //arived
       cout<<"dubin final ends."<< endl;
       SendControlToDrone( ControlCommand(0,0,0,0) );
     }
     else
     { //otherwise, just jump to the next dubin's curve
       ++idx_dubin;
       result= DubinCommand(dubin_segs[idx_dubin], _t_limit);
     }//else ends
   }//end result==1
   
   if(result==0)
   {//time up
     if_reach= 1;
     //here we may need to publish a state
   }

   if(if_reach==1||if_reach==2)
   {
     //for the next path
     if_receive= false;
     path_msg.dubin_path.clear();
     //for the next if_new_path
     if_new_path= false;
     if_new_rec= false; 
     SetRestartDefault();
   }//set to default
   
   return 0;
}//PathCommand() ends

int ParrotExe::DubinCommand(DubinSeg& db_seg, const double _t_limit)
{  //0:time up, 1:cfg_stop reached, 2: seg end reached, -1: still ongoing
   //if already at the target, just return
   //cout<<"DubinCommand"<< endl;
   QuadCfg cfg_stop= db_seg.cfg_stop;
   quadDubins3D dubin_3d= db_seg.d_dubin;
   
   if( sqrt(pow(x_est-cfg_stop.x,2)+pow(y_est-cfg_stop.y,2)+pow(z_mea-cfg_stop.z,2))<end_r )
   {
     if_restart_dubin= true;
     std::cout<<"x_est: "<<x_est<<" y_est: "<<y_est<<" z_mea: "<<z_mea<<" yaw_est: "<<yaw_est/M_PI*180.<<std::endl;
     return 1;
   }
   //or see which segment the quad is closest to
   //find the closest curve seg
   if(if_restart_dubin)
   {
     if_restart_dubin= false;
     
     /***print the dubin******/
     std::cout<<"start: "<<dubin_3d.cfg_start.x<<" "<<dubin_3d.cfg_start.y<<" "<<dubin_3d.cfg_start.z<<" "<<dubin_3d.cfg_start.theta*180./M_PI<<std::endl;
     std::cout<<"i1: "<<dubin_3d.cfg_i1.x<<" "<<dubin_3d.cfg_i1.y<<" "<<dubin_3d.cfg_i1.z<<" "<<dubin_3d.cfg_i1.theta*180./M_PI<<std::endl;
     std::cout<<"i2: "<<dubin_3d.cfg_i2.x<<" "<<dubin_3d.cfg_i2.y<<" "<<dubin_3d.cfg_i2.z<<" "<<dubin_3d.cfg_i2.theta*180./M_PI<<std::endl;
     std::cout<<"end: "<<dubin_3d.cfg_end.x<<" "<<dubin_3d.cfg_end.y<<" "<<dubin_3d.cfg_end.z<<" "<<dubin_3d.cfg_end.theta*180./M_PI<<std::endl;
     std::cout<<"stop: "<<cfg_stop.x<<" "<<cfg_stop.y<<" "<<cfg_stop.z<< std::endl;
     /***print ends******/
     
     QuadCfg cfgs[]={dubin_3d.cfg_start,dubin_3d.cfg_i1,dubin_3d.cfg_i2,dubin_3d.cfg_end};
     double d[4];
    
     int idx= -1,idx_seg;
     double dis_temp = 1e7;
     
     for(int i=0;i<4;++i)
     {
	double dis= pow(cfgs[i].x-x_est,2)+pow(cfgs[i].y-y_est,2)+pow(cfgs[i].z-z_mea,2);
	d[i]= sqrt(dis);
	
	if( dubin_3d.seg_param[i]==0) 
	{
	  continue;
	}
	if(d[i]<dis_temp)
	{
	   dis_temp= d[i];
	   idx= i;
	}//if dis ends
     }//for int i ends
     
     if(idx==1||idx==2)
     {
       if( d[idx-1]/(dubin_3d.seg_param[idx-1]+1e-10)<d[idx+1]/(dubin_3d.seg_param[idx]+1e-10) )
	  idx_seg= idx-1;
       else
	  idx_seg= idx;
     }
     else
       idx_seg= idx;//if idx ends
     if(idx_seg==3)
       idx_seg=2;
     //make sure idx_seg correct value
     if(!(idx_seg==0||idx_seg==1||idx_seg==2))
     {
       try {
        throw std::runtime_error ("idx_seg must be 0 or 1 or 2");
       }
       catch (std::runtime_error &e) {
        std::cout << "Caught a runtime_error exception: "
                  << e.what () << '\n';
       }
     }
     idx_dubin_sub= idx_seg;

   }//if_restart_dubin ends
   else
   {
     if(idx_dubin_sub== -1)
     {
       try {
        throw std::runtime_error ("error:idx_dubin_sub unassigned");
       }
       catch (std::runtime_error &e) {
        std::cout << "Caught a runtime_error exception: "
                  << e.what () << '\n';
       }
     }
   }//else ends
   
   double t_limit = _t_limit;
   int db_result= SegCommand(db_seg,idx_dubin_sub,t_limit);
   //time up
   if(db_result==0)
   {
     if_restart_dubin= true;
     std::cout<<"dubin time up"<<std::endl;
   }
   //cfg_stop reached
   if(db_result==1)
   {
     if_restart_dubin= true;
     std::cout<<"dubin stop point reached"<<std::endl;
     //return db_result;
   }
   //segment ends
   if(db_result==2)
   {
     if(idx_dubin_sub==2) 
     {	//dubin ends actually
       if_restart_dubin= true;
       //db_result=1;
       std::cout<<"dubin end reached"<<std::endl;
	//return db_result;
     }
     else
     {
       ++idx_dubin_sub;
       //we need to start from a new nonzero segment
       while(idx_dubin_sub<3 && dubin_3d.seg_param[idx_dubin_sub]==0)
          ++idx_dubin_sub;
       std::cout<<"idx_dubin_sub: "<<idx_dubin_sub<<std::endl;
       //while ends
       if(idx_dubin_sub <3)
       {
         cout<<"budo,budo"<< endl;
	 db_result= SegCommand(db_seg,idx_dubin_sub,t_limit);
       }
     }
   }
   return db_result;
   //0:time up, 1:cfg_stop reached, 2: seg end reached, -1: still ongoing
}//DubinCommand ends

int ParrotExe::SegCommand(DubinSeg& db_seg, int idx_sub, double _t_limit)
{ //-1: on the fly, 0:time up, 1:target reached, 2:length reached
   //cout<<"SegCommand"<< endl;
   if(if_restart_seg) 
   {
     if_restart_seg= false;
     x_start= x_est;
     y_start= y_est;
     z_start= z_mea;
     d_length= 0.0; 
   }//if_restart_seg ends
   int seg_result= -1;
   //initial criterion
   //if time limit reached?
   if( ros::Time::now()-t_start>= ros::Duration(_t_limit) ) 
   { 
      seg_result= 0;
      cout<<"time up:"<< ros::Time::now().toSec()-t_start.toSec()<<" "<< ros::Duration(_t_limit).toSec() <<endl;
      if_restart_seg= true;
      return seg_result;
      //break;
   }
   
   quadDubins3D dubin_3d= db_seg.d_dubin;
   QuadCfg cfg_stop= db_seg.cfg_stop;
   
   if( !(idx_sub==0||idx_sub==1||idx_sub==2) )
    //std::runtime_error("idx_sub must be 0 or 1 or 2.");
   {
     try {
        throw std::runtime_error ("idx_sub must be 0 or 1 or 2.");
     }
     catch (std::runtime_error &e) {
        std::cout << "Caught a runtime_error exception: "
                  << e.what () << '\n';
     }
   }
   const int* types = DIRDATA[dubin_3d.path2D.type];
   int type= types[idx_sub];
   if(!( type == L_SEG || type == S_SEG || type == R_SEG ))
     //std::runtime_error("dubin_sub type incorrect");
   {
     try {
        throw std::runtime_error ("dubin_sub type incorrect");
     }
     catch (std::runtime_error &e) {
        std::cout << "Caught a runtime_error exception: "
                  << e.what () << '\n';
     }
   }
   
   //the start and end point of this seg
   QuadCfg cfg_start,cfg_end;
   //get tne start and end
   if(idx_sub==0){
      cfg_start= dubin_3d.cfg_start;
      cfg_end= dubin_3d.cfg_i1;
   }
   else if(idx_sub==1){
      cfg_start= dubin_3d.cfg_i1;
      cfg_end= dubin_3d.cfg_i2;
   }
   else{
      cfg_start= dubin_3d.cfg_i2;
      cfg_end= dubin_3d.cfg_end;
   }//if idx_s

   //different distances
   double s_init= dubin_3d.CloseLength(x_start,y_start,z_start);
   double s_target= dubin_3d.CloseLength(cfg_stop.x,cfg_stop.y,cfg_stop.z);
   double s_end= dubin_3d.CloseLength(cfg_end.x,cfg_end.y,cfg_end.z);
   
   arma::vec::fixed<3> v_quad,v_target,v_end;

   //to calculate the length travelled along the seg
   d_length+= sqrt(pow(x_pre-x_est,2)+pow(y_pre-y_est,2)+pow(z_pre-z_mea,2));
   //assign previous coordinates
   x_pre= x_est;
   y_pre= y_est;
   z_pre= z_mea; 

   //v_quad
   v_quad<< vx_est<< vy_est<< vzm_est; 

   //if cfg_stop reached?
   double target_dis=sqrt(pow(cfg_stop.x-x_est,2)+pow(cfg_stop.y-y_est,2)+pow(cfg_stop.z-z_mea,2));
   
   v_target<<cfg_stop.x-x_est<<cfg_stop.y-y_est<<cfg_stop.z-z_mea;
	    
   if( dot(v_quad,v_target)<=0&&target_dis<= 3*end_r ||target_dis<end_r
     ||dot(v_quad,v_target)<=0&&d_length>s_target-s_init
     ) 
   {
      std::cout<<"x_est: "<<x_est<<" y_est: "<<y_est<<" z_mea: "<<z_mea<<" yaw_est: "<<yaw_est*180/M_PI<< std::endl;
      
      if(dot(v_quad,v_target)<=0 && d_length>s_target-s_init)
      {
         std::cout<<"target length reached"<<std::endl;
      }
      else if(dot(v_quad,v_target)<=0&&target_dis<= 3*end_r)
         std::cout<<"target pass reached"<<std::endl;
      else
	 std::cout<<"target just reached"<<std::endl;     

      seg_result= 1;
      if_restart_seg= true;
      return seg_result;
      //break;
   }

   //if end of this seg reached?
   double end_dis=sqrt(pow(cfg_end.x-x_est,2)+pow(cfg_end.y-y_est,2)+pow(cfg_end.z-z_mea,2));
   v_end<<cfg_end.x-x_est<<cfg_end.y-y_est<<cfg_end.z-z_mea;  
   if( dot(v_quad,v_end)<=0&&end_dis<= 3*end_r || end_dis<=end_r
     ||dot(v_quad,v_end)<=0 && d_length> s_end-s_init 
     ||d_length>1.5*(s_end-s_init) ) 
   {
     //std::cout<<" end reached" <<std::endl;
     std::cout<<"x_est: "<<x_est<<" y_est: "<<y_est<<" z_mea: "<<z_mea<<" yaw_est: "<<yaw_est*180/M_PI<< std::endl;
     
     if(dot(v_quad,v_end)<=0&&end_dis<=3*end_r)
       std::cout<<"end pass reached"<<std::endl;
     else if( end_dis<=end_r )
       std::cout<<"end just reached"<<std::endl;
     else if(dot(v_quad,v_end)<=0 && d_length> s_end-s_init)
       std::cout<<"end length reached"<<std::endl;
     else
       std::cout<<"end length limited"<<std::endl;
     
     seg_result= 2;
     if_restart_seg= true;
     return seg_result;
   }
  
   //controlling part
   if(type== L_SEG||type== R_SEG)
   {
     CircleStepCommand(cfg_start,cfg_end,type,rho);
   }//if type==L_SEG or R_SEG ends
   else //type== S_SEG
   {
     LineStepCommand(cfg_start,cfg_end);   
   }//else ends
   
   //if time limit reached?
   if( ros::Time::now()-t_start>= ros::Duration(_t_limit) ) 
   { 
      seg_result= 0;
      if_restart_seg= true;
   }

   return seg_result; 
} //SegCommand ends

int ParrotExe::StepCommand(const arma::vec::fixed<3> u,double dt)
{
   double de_yaw= atan2(u(1),u(0) );
   cout<<"StepCommand: "<<"u(0): "<<u(0)<<" u(1): "<<u(1)<<" u(2): "<<u(2)<<endl;
   cout<<"    position: "<< x_est<<" "<<y_est<<" "<<z_mea<<endl;
   cout<<"    speed: "<< vx_est<<" "<<vy_est<< endl;
   yaw_est = jesus_library::mapAnglesToBeNear_PIrads( yaw_est, de_yaw);
   controlMid.setFeedback( x_est, y_est, vx_est, vy_est, yaw_est, z_mea);
   controlMid.setReference( 0.0, 0.0, de_yaw, 0.0, u(0), u(1) );
   //double v= sqrt( u(0)*u(0)+u(1)*u(1) );
   //controlMid.setReference(0.0,0.0,de_yaw,0.0,v,0.0);
   //print and test
   controlMid.getOutput( &pitchco, &rollco, &dyawco, &dzco);
   cout<<"    out: "<<pitchco<<" "<<rollco<<" "<<dzco<<" "<<dyawco<<endl;

   SendControlToDrone( ControlCommand( pitchco, rollco, u(2), dyawco ) );
   //last dt
   cout<<"dt: "<< dt << endl;
   ros::Duration(dt).sleep();
   return 0;
}//StepCommand ends

int ParrotExe::StepResponse(const arma::vec::fixed<3> u, geometry_msgs::Twist& c_twist)
{
    double de_yaw= atan2(u(1),u(0) );
    yaw_est = jesus_library::mapAnglesToBeNear_PIrads( yaw_est, de_yaw);
    controlMid.setFeedback( x_est, y_est, vx_est, vy_est, yaw_est, z_mea);
    controlMid.setReference( 0.0, 0.0, de_yaw, 0.0, u(0), u(1) );
    controlMid.getOutput( &pitchco, &rollco, &dyawco, &dzco);
    //assign
    c_twist = geometry_msgs::Twist();
    c_twist.linear.x= -pitchco;
    c_twist.linear.y= rollco;
    c_twist.linear.z= u(2);
    c_twist.angular.z= dyawco;
    return 0;
}//StepResponse ends

int ParrotExe::CircleStepCommand(const QuadCfg& cfg_start,const QuadCfg& cfg_end,int type,double rho)
{
   double lambda_h = 1.;
   double vec[]={0};
   if(type==L_SEG)
   { 
      vec[0]= -sin(cfg_start.theta);
      vec[1]= cos(cfg_start.theta);
   }
   else
   {
      lambda_h= -1;
      vec[0]= sin(cfg_start.theta);
      vec[1]= -cos(cfg_start.theta);
   }
   
   double center[3]= {cfg_start.x+rho*vec[0],cfg_start.y+rho*vec[1],cfg_start.z};
   double c_n= center[0];
   double c_e= center[1];
   double c_d= center[2];
    
   double de_length= fabs(cfg_end.theta-cfg_start.theta)*rho;
   //std::cout<<"de_length= "<<de_length<<std::endl;
   double h_diff= cfg_end.z-cfg_start.z;
   double tan_gamma= h_diff/de_length;
   //to calculate the desired yaw
   //Rxy is the vector pointed to the current position from the center
   //Vxy is the quad's velocity vector in xy plan
   arma::vec::fixed<3> Rxy, Vxy, Dxy;
   Rxy<< x_est-c_n << y_est-c_e << 0;
   Vxy<< vx_est << vy_est << 0;
   Dxy = cross(Vxy, Rxy);
   Dxy = cross(Rxy, Dxy);
   //double d_yaw= atan2(Dxy(1), Dxy(0) );
   //vectors for calculation
   arma::vec::fixed<3> u, u1, u2, fc1, fc2;
   //calculate starts
   double r_n= x_est; 
   double r_e= y_est;
   double r_d= z_mea;
		    
   double pacpr[3]={0,0,0};
   pacpr[0]= 2*(r_n-c_n)/(rho*rho);
   pacpr[1]= 2*(r_e-c_e)/(rho*rho); 
    
   double pappr[3]={0,0,0};
   double numer= pow(r_n-c_n,2)+pow(r_e-c_e,2);
   pappr[0]= tan_gamma/lambda_h*(r_e-c_e)/numer;
   pappr[1]= -tan_gamma/lambda_h*(r_n-c_n)/numer;
   pappr[2]= 1./rho;
    
   double ac= pow((r_n-c_n)/rho,2)+pow((r_e-c_e)/rho,2)-1;
   double phi_h= atan2(cfg_start.y-c_e, cfg_end.x-c_n);
   double ap= (r_d-c_d)/rho-tan_gamma/lambda_h*(atan2(r_e-c_e,r_n-c_n)-phi_h ); 
   fc1<<pacpr[0] <<pacpr[1] <<pacpr[2];
   fc2<<pappr[0] <<pappr[1] <<pappr[2];
   u1<< ac*pacpr[0]+ap*pappr[0]\
     << ac*pacpr[1]+ap*pappr[1]\
     << ac*pacpr[2]+ap*pappr[2];
   u2= -lambda_h*cross(fc1,fc2);
   u= -K1*u1+K2*u2;
   //double u_mag= sqrt(u(0)*u(0)+u(1)*u(1)+u(2)*u(2));
   double uxy_mag= sqrt( u(0)*u(0)+u(1)*u(1) );
   double uz_mag= fabs(u(2));
   
   if(uxy_mag!=0 && uz_mag!=0)
   {
     double cons= min(this->v/uxy_mag,this->vz/uz_mag);
     u<< u(0)*cons<< u(1)*cons << u(2)*cons;  
   }
   else if(uxy_mag==0 && uz_mag!=0)
     u<<0.<<0.<<vz/uz_mag;
   else if(uxy_mag!=0 && uz_mag==0)
   {
     double cons= this->v/uxy_mag;
     u<< u(0)*cons<< u(1)*cons<< 0;
   }
   else 
     u<<0.<<0.<<0.;
   //step control
   if(u(0)+u(1)+u(2)!=0 )
     StepCommand(u,dt);  
}//CircleStepCommand ends

int ParrotExe::LineStepCommand(const QuadCfg& cfg_start, const QuadCfg& cfg_end)
{
    double x_start= cfg_start.x;
    double y_start= cfg_start.y;
    double z_start= cfg_start.z;
    double x_end= cfg_end.x;
    double y_end= cfg_end.y;
    double z_end= cfg_end.z;
    
    double dis= sqrt(pow(x_start-x_end,2)+pow(y_start-y_end,2)+pow(z_start-z_end,2));
    //d_yaw
    //double d_yaw= atan2(cfg_end.y-y_est,cfg_end.x-x_est);

    double phi= atan2(y_end-y_start,x_end-x_start);
    double gam= asin( (z_end-z_start)/dis );

    arma::vec::fixed<3> n_lon,n_lat,u;

    n_lon<<-sin(phi)<<cos(phi)<<0.;
    n_lat<<cos(phi)*sin(gam)<<sin(phi)*sin(gam)<<-cos(gam);
    
    double a_lon= n_lon(0)*(x_est-x_start)+n_lon(1)*(y_est-y_start)+n_lon(2)*(z_mea-z_start);
    double a_lat= n_lat(0)*(x_est-x_start)+n_lat(1)*(y_est-y_start)+n_lat(2)*(z_mea-z_start);
    //unnormailised velocity
    u = -K1*(a_lon*n_lon+a_lat*n_lat)+K2*cross(n_lat,n_lon);
    //double u_mag= sqrt(u(0)*u(0)+u(1)*u(1)+u(2)*u(2));
    double uxy_mag= sqrt( u(0)*u(0)+u(1)*u(1) );
    double uz_mag= fabs(u(2));
    
    if(uxy_mag!=0 && uz_mag!=0)
    {
      double cons= min(this->v/uxy_mag,this->vz/uz_mag);
      u<< u(0)*cons<< u(1)*cons << u(2)*cons;  
    }
    else if(uxy_mag==0 && uz_mag!=0)
      u<<0.<<0.<<vz/uz_mag;
    else if(uxy_mag!=0 && uz_mag==0)
    {
      double cons= this->v/uxy_mag;
      u<< u(0)*cons<< u(1)*cons<< 0;
    }
    else 
      u<<0.<<0.<<0.;
    //step control
    if(u(0)+u(1)+u(2)!=0 )
      StepCommand(u,dt);      
      
}//LineStepCommand ends

int ParrotExe::CircleCommand(const QuadCfg& start,const QuadCfg& end,int type,double rho,double _t_limit)
{
   if(if_restart_seg)
    {
      if_restart_seg= false;
      x_start= x_est;
      y_start= y_est;
      z_start= z_mea;
      d_length= 0.0; 
    }
    int seg_result= -1;
    //if time limit reached?
    if( ros::Time::now()-t_start>= ros::Duration(_t_limit) ) 
    { 
       cout<<"now: "<<ros::Time::now().toSec()<<" t_start: "<<t_start.toSec()<<endl;
       seg_result= 0;
       if_restart_seg= true;
       return seg_result;
       //break;
    }
    //the total length
    double t_len= fabs(end.theta-start.theta)*rho;
    
    arma::vec::fixed<3> v_quad, v_end;
    //to calculate the length travelled along the seg
    double d_len= sqrt(pow(x_pre-x_est,2)+pow(y_pre-y_est,2)+pow(z_pre-z_mea,2));
    d_length+= d_len;
    
    //assign previous coordinates
    x_pre= x_est;
    y_pre= y_est;
    z_pre= z_mea; 

    //v_quad
    v_quad<< vx_est<< vy_est<< vzm_est; 
    //if the length reached?
    double end_dis=sqrt(pow(end.x-x_est,2)+pow(end.y-y_est,2)+pow(end.z-z_mea,2));
    //v_end
    v_end<< end.x-x_est<< end.y-y_est<< end.z-z_mea; 
       
    //if the end is reached?
    if( dot(v_quad,v_end)<=0&&end_dis<= 3*end_r || end_dis<=end_r
      ||dot(v_quad,v_end)<=0 && d_length> t_len 
      ||d_length>1.5*t_len 
      )
    {
      seg_result= 2;
      if_restart_seg= true;
      cout<<"x_est: "<<x_est<<" y_est: "<<y_est<<" z_mea: "<<z_mea<<" yaw_est: "<<yaw_est*180/M_PI<<endl;
      if( dot(v_quad,v_end)<=0&&end_dis<= 3*end_r || end_dis<=end_r)
      {
	cout<<"circle end reached"<< endl;
      }
      else if( dot(v_quad,v_end)<=0 && d_length> t_len)
      {
	cout<<"end length: "<<d_length << endl;
      }
      else
      {
	cout<<"pure length: "<<d_length<<" t_len: "<<t_len<< endl;
      }
      return seg_result;
    }
    //control part
    CircleStepCommand(start,end,type,rho);
    //if time limit reached?
    if( ros::Time::now()-t_start>= ros::Duration(_t_limit) ) 
    { 
      seg_result= 0;
      if_restart_seg= true;
    }

    return seg_result; 

}//CircleCommand

int ParrotExe::LineCommand(const QuadCfg& start,const QuadCfg& end, double _t_limit)
{
   if(if_restart_seg)
   {
     if_restart_seg= false;
     x_start= x_est;
     y_start= y_est;
     z_start= z_mea;
     d_length= 0.0; 
   }
   int seg_result= -1;
   //if time limit reached?
   if( ros::Time::now()-t_start>= ros::Duration(_t_limit) ) 
   { 
      cout<<"now: "<<ros::Time::now().toSec()<<" t_start: "<<t_start.toSec()<<endl;
      seg_result= 0;
      if_restart_seg= true;
      return seg_result;
      //break;
   }
   //total length
   double t_len= sqrt(pow(start.x-end.x,2)+pow(start.y-end.y,2)+pow(start.z-end.z,2) );
      
   arma::vec::fixed<3> v_quad, v_end;
   //to calculate the length travelled along the seg
   double d_len= sqrt(pow(x_pre-x_est,2)+pow(y_pre-y_est,2)+pow(z_pre-z_mea,2));
   d_length+= d_len;
   
   //assign previous coordinates
   x_pre= x_est;
   y_pre= y_est;
   z_pre= z_mea; 

   //v_quad
   v_quad<< vx_est<< vy_est<< vzm_est; 
   //if the length reached?
   double end_dis=sqrt(pow(end.x-x_est,2)+pow(end.y-y_est,2)+pow(end.z-z_mea,2));
   //v_end
   v_end<< end.x-x_est<< end.y-y_est<< end.z-z_mea; 
      
   //if the end is reached?
   if( dot(v_quad,v_end)<=0&&end_dis<= 3*end_r || end_dis<=end_r
     ||dot(v_quad,v_end)<=0 && d_length> t_len 
     ||d_length>1.5*t_len 
     )
   {
     seg_result= 2;
     if_restart_seg= true;
     cout<<"x_est: "<<x_est<<" y_est: "<<y_est<<" z_mea: "<<z_mea<<" yaw_est: "<<yaw_est*180/M_PI <<endl;
     if( dot(v_quad,v_end)<=0&&end_dis<= 3*end_r || end_dis<=end_r)
     {
       cout<<"line end reached"<< endl;
     }
     else if( dot(v_quad,v_end)<=0 && d_length> t_len)
     {
       cout<<"end length: "<<d_length << endl;
     }
     else
     {
       cout<<"pure length: "<<d_length<<" t_len: "<<t_len<< endl;
     }
     return seg_result;
   }
   //the control part
   LineStepCommand(start,end);

   //if time limit reached?
   if( ros::Time::now()-t_start>= ros::Duration(_t_limit) ) 
   { 
      seg_result= 0;
      if_restart_seg= true;
   }

   return seg_result; 

}//LineCommand ends

};//namespace ends
