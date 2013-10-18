#include "ParrotExe.hpp"
#include "ticpp.h"
#include "ros/ros.h"
#include "armadillo"
//#include "dubins.h"
#include "fstream"
//controller
#include "controller/midlevelCnt/Controller_MidLevelCnt.h"
#include "controller/midlevelCnt/Controller_MidLevel_controlModes.h"
#include "Other/jesus_library.h"

//function to send command to the parrot
void SendControlToDrone(ControlCommand cmd); 

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

ParrotExe::ParrotExe():log_file("ardrone_path_rec.txt")
{
   //flags default
   if_receive= false;
   if_reach= 0;
   if_new_path= false;
   if_new_rec= false;
   if_joy= false;
   uav_state_idx= -1;
   //publishers
   //for planning related
   pub_rec = nh.advertise<std_msgs::Bool>("path_rec",1);
   pub_vel = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
   pub_reach= nh.advertise<std_msgs::Bool>("quad_reach",1);
   pub_state= nh.advertise<ardrone_rrt_avoid::QuadState_msg>("quad_state",1); 
   pub_new_rec= nh.advertise<std_msgs::Bool>("if_new_rec",1);
   //for quad related
   takeoff_pub= nh_.advertise<std_msgs::Empty>("ardrone/takeoff",1);
   land_pub= nh_.advertise<std_msgs::Empty>("ardrone/land",1);

   //Subscribers
   sub_path = nh.subscribe("path", 1, &ParrotExe::pathCallback,this);
   sub_if_new= nh.subscribe("if_new_path",1, &ParrotExe::newCallback,this);
   //specify some parameters of the quad
   if(ParamFromXML("/home/yucong/.ros/param.xml")!=0)
     std::runtime_error("ParamFromXML error");
   //parameters
   rho= v/yaw_rate;
   speed= sqrt(v*v+vz*vz);
   end_r= max(speed*dt,0.2);
   //controller related
   tkm1 = ros::Time::now();
   tk = ros::Time::now();
   elapsed_time = tk - tkm1;
   elapsed_time_dbl = elapsed_time.sec + ( (double) elapsed_time.nsec)/1E9;
   x_est = 0.0, y_est = 0.0;
   vxm_est = 0.0, vym_est = 0.0, yaw_est = 0.0; // meter/s
   yawci = 0.0,  vxfi = 0.0, vyfi = 0.0, dzfi=0.; // meter/s
   pitchco = 0.0, rollco = 0.0, dyawco = 0.0, dzco = 0.0;
   // For trajectory control
   controlMid.setControlMode(Controller_MidLevel_controlMode::TROJECTORY_CONTROL);
}

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
   tk = navdataPtr->header.stamp;
   elapsed_time = tk - tkm1;
   elapsed_time_dbl= elapsed_time.sec+ (double)elapsed_time.nsec/1E9;
   vxm_est = (double)navdataPtr->vx/1000.0; 
   vym_est = (double)navdataPtr->vy/1000.0;
   yaw_est = (double)navdataPtr->rotZ*M_PI/180;
   vx_est = vxm_est*cos(yaw_est) - vym_est*sin(yaw_est); 
   vy_est = vxm_est*sin(yaw_est) + vym_est*cos(yaw_est); 
   x_est += elapsed_time_dbl * vx_est;
   y_est += elapsed_time_dbl * vy_est;
   z_mea = navdataPtr->altd/1000.;
   //which state the quad is
   uav_state_idx= navdataPtr->state;
}//navdataCb ends

void ParrotExe::joyCb(const sensor_msgs::JoyConstPtr joy_msg)
{
    if(joy_msg->axes.size() < 4) {
        ROS_WARN_ONCE("Error: Non-compatible Joystick!");
        return;
    }
    // Avoid crashes if non-ps3 joystick is being used
    short unsigned int L1 = 4;
    short unsigned int R1 = 5;
    int YAW = 3;
    int GAZ = 4;
    int ROLL = 0;
    int PITCH = 1;
    
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
        //cout << "start control" << endl;
    }
    // are we actually controlling with the Joystick?
    if( justStartedControlling || if_joy )
    {
 	ControlCommand c;
	c.yaw = joy_msg->axes[YAW];
	c.gaz = joy_msg->axes[GAZ];
	c.roll = -joy_msg->axes[ROLL];
	c.pitch = -joy_msg->axes[PITCH];

	sendControlToDrone(c);
	//cout << lastL1Pressed << " " << joy_msg->buttons.at(L1) << endl;
        if( joy_msg->buttons.at(L1) )
	{
          if( uav_state_idx == 2 ){
            sendTakeoff();
	  }
	  if( uav_state_idx == 3 || uav_state_idx == 7 ){
	    sendControlToDrone( ControlCommand(0,0,0,0) );
	    sendLand();
	  }
          if( uav_state_idx == 4 )
	    sendLand();
	}//if joy_msg ends

        if(!lastR1Pressed && joy_msg->buttons.at(R1))
			sendToggleState();
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

void ParrotExe::SendControlToDrone(ControlCommand cmd)
{
   // TODO: check converstion (!)
   geometry_msgs::Twist cmdT;
   cmdT.angular.z = -cmd.yaw;
   cmdT.linear.z = cmd.gaz;
   cmdT.linear.x = -cmd.pitch;
   cmdT.linear.y = -cmd.roll;
   pub_vel.publish(cmdT);
}

void ParrotExe::sendLand()
{
   land_pub.publish(std_msgs::Empty());
   //cout << "land land" << endl;
}

void ParrotExe::sendTakeoff()
{
   takeoff_pub.publish(std_msgs::Empty());
   //cout << "take off" << endl;
}

int ParrotExe::CommandParrot(const double _t_limit)
{   //if no path, just stop and hover
   if(if_restart && path_msg.dubin_path.size()==0 )
   { //command it to stop. for fixed wing, maybe other mechnism
     twist.linear.x=0;
     twist.linear.y=0;
     twist.linear.z=0;
     twist.angular.z=0;
     pub_vel.publish(twist);
     //require a new path
     if_receive= false;
     if_new_path= false;
     if_new_rec= false; 

     cout<<"no path,stop"<<endl;
     return 0;
   }
   
   //first see if the current position is already at the goal
   yucong_rrt_avoid::DubinSeg_msg dubin_last = path_msg.dubin_path.back();
   if( sqrt(pow(x_est-dubin_last.stop_pt.x,2)+pow(y_est-dubin_last.stop_pt.y,2)+pow(z_mea-dubin_last.stop_pt.z,2)) < end_r )
   {
     twist.linear.x=0;
     twist.linear.y=0;
     twist.linear.z=0;
     twist.angular.z=0;
     pub_vel.publish(twist);
     if_reach= 2;
     cout<<"got target already"<<endl;
     return;
   }
   
   if(if_restart)
   {
     t_start= ros::Time::now();
     //if none of the above happens just convert msg to vector of DubinSegs
     if(!dubin_segs.empty() ) dubin_segs.clear();
     for(int i=0;i!=path_msg.dubin_path.size();++i)
     {
       yucong_rrt_avoid::DubinSeg_msg db_msg= path_msg.dubin_path[i];
       QuadCfg start(db_msg.d_dubin.start.x,db_msg.d_dubin.start.y,db_msg.d_dubin.start.z,db_msg.d_dubin.start.theta);
       QuadCfg end(db_msg.d_dubin.end.x,db_msg.d_dubin.end.y,db_msg.d_dubin.end.z,db_msg.d_dubin.end.theta);
       quadDubins3D db_3d(start,end,rho);
       //std::cout<<i<<" seg length: "<<db_3d.seg_param[0]<<" "<<db_3d.seg_param[1]<<" "<<db_3d.seg_param[2]<<std::endl;
       QuadCfg stop(db_msg.stop_pt.x,db_msg.stop_pt.y,db_msg.stop_pt.z,0);
       //std::cout<<"to msg stop: "<<stop.x<<" "<<stop.y<<" "<<stop.z<<std::endl;
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
	  double dis=pow(start.x-x_c,2)+pow(start.y-y_c,2)+pow(start.z-z_c,2);
	  dwp[i]= sqrt(dis);
	  if(dwp[i]<dis_temp )
	  {
	    dis_temp= dwp[i];
	    idx= i;
	  }//if dis ends

	  //if( i!=dubin_segs.size()-1 )
	  {
	    QuadCfg stop= dubin_segs[i].cfg_stop; 
	    len_wp[i]= dubin_segs[i].d_dubin.CloseLength(stop.x,stop.y,stop.z);
	  }
       }//for int i ends
       //the final stop point
       QuadCfg stop= dubin_segs.back().cfg_stop;
       double dis=pow(stop.x-x_c,2)+pow(stop.y-y_c,2)+pow(stop.z-z_c,2);
       dis= sqrt(dis);
       dwp[dubin_segs.size()]= dis;
       if(dis<dis_temp) idx= dubin_segs.size();
	     
       if( idx!=0 && idx!=dubin_segs.size() )
       {
	 if( dwp[idx-1]/len_wp[idx-1]< dwp[idx+1]/len_wp[idx] )
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
     idx_dubin= idx_sec;
     if_start= false;
       //then see which seg of the dubins curve it should follow
   }//if_restart ends
   else{
     if(idx_dubin== -1)
       std::runtime_error("error:idx_dubin unassigned");
   }//else ends
   
   int result= DubinCommand(dubin_segs[idx_dubin], _t_limit);
   if(result== 1)
   {
     if(idx_dubin== dubin_segs.size()-1) if_reach==2; //arived
     else{
       ++idx_dubin;
       result= DubinCommand(dubin_segs[idx_dubin], _t_limit);
     }//else ends
   }//end result==1
   //arrival criterion here
}//CommandParrot() ends

int ParrotExe::DubinCommand(DubinSeg& db_seg, const double _t_limit)
{
   //if already at the target, just return
   QuadCfg cfg_stop= db_seg.cfg_stop;
   if( sqrt(pow(x_est-cfg_stop.x,2)+pow(y_est-cfg_stop.y,2)+pow(z_mea-cfg_stop.z,2))<end_r )
   return 1;

}//DubinCommand ends

