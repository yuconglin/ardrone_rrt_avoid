#ifndef PATHEXE_H
#define PATHEXE_H
//includes
#include "ardrone_autonomy/Navdata.h"
//ros related
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int16.h"
#include <geometry_msgs/Twist.h>
#include "std_srvs/Empty.h"

#include "quadDubins3D.h"
#include "QuadCfg.h"
#include <fstream>
#include "armadillo"

//ros msgs
#include "ardrone_rrt_avoid/DubinPath_msg.h"
#include "ardrone_rrt_avoid/QuadState_msg.h"

//declaration of classes used
class Controller_MidLevelCnt;

namespace Ardrone_rrt_avoid{ 
class ParrotExe{
   //struct 
   struct ControlCommand
   {  //x,y,yaw, reverse
     inline ControlCommand() {roll = pitch = yaw = gaz = 0;}
     inline ControlCommand( double pitch, double roll, double gaz, double yaw ) 	 {
	  this->roll = roll;
	  this->pitch = pitch;
	  this->yaw = yaw;
	  this->gaz = gaz;
     }
     double yaw, roll, pitch, gaz;
   };
 
 public:
   //struct
   struct DubinSeg
   {
     quadDubins3D d_dubin;
     QuadCfg cfg_stop;
   };

   //constructor
   ParrotExe(Controller_MidLevelCnt& _controlMid,char* file_nav="ardrone_log_nav.txt");
   //read basic params from xml file
   int ParamFromXML(const char* pFilename="/home/yucong/fuerte_workspace/sandbox/yucong_rrt_avoid/src/common/param.xml");
   //callback functions 
   void pathCallback(const ardrone_rrt_avoid::DubinPath_msg::ConstPtr& msg);
   void newCallback(const std_msgs::Bool::ConstPtr& msg);
   void navdataCb(const ardrone_autonomy::NavdataConstPtr navdataPtr);
   void joyCb(const sensor_msgs::JoyConstPtr joy_msg);
   //to publish flags
   void PublishFlags();
   //to publish commands
   void SendControlToDrone(ControlCommand cmd);
   void SendCommand(double cx,double cy,double cz,double cw);
   //to take off or land
   void sendLand();
   void sendTakeoff();
   void sendStop();
   void sendEmergencyStop();
   void sendFlattrim();
   //command the Parrot
   //command to execute a whole path
   int PathCommand(const double _t_limit);
   //command to execute a dubin's curve
   int DubinCommand(DubinSeg& db_seg, const double _t_limit);
   //command to execute a segment of dubin line/circle
   int SegCommand(DubinSeg& db_seg, int idx_sub, double _t_limit);
   //command a step with velocity input u,time interval dt
   int StepCommand(const arma::vec::fixed<3> u,double d_yaw,double dt);
   //command to execute a line.
   int LineStepCommand(const QuadCfg& start,const QuadCfg& end);
   //command to execute a circle curve
   int CircleStepCommand(const QuadCfg& start,const QuadCfg& end,int type,double rho);
   //for test
   int LineCommand(const QuadCfg& start,const QuadCfg& end, double _t_limit); 
   int CircleCommand(const QuadCfg& start,const QuadCfg& end,int type,double rho,double _t_limit);
   
   //to publish quad's state
   void PubQuadState();  
   //to access flags
   inline bool GetIfRec(){return this->if_receive;}
   inline bool GetIfReach(){return this->if_reach;}
   inline bool GetIfNewPath(){return this->if_new_path;}
   inline int GetUavStateIdx(){return this->uav_state_idx;}
   inline bool GetIfJoy(){return this->if_joy;}
   inline double GetStartTimeSec(){return this->t_start.toSec();}
   int GetCurrentCfg(QuadCfg& cfg);
   inline ros::NodeHandle* GetHandlePt(){return &nh;} 
   //set init time
   inline void SetInitTime(ros::Time _t_now) {this->t_init= _t_now.toSec(); }
   inline void SetStartTime(ros::Time _t_now){this->t_start= _t_now;}
   inline void SetYawInit(double _yaw){this->YawInit= _yaw;}
   //set some basic elements
   inline void SetXEst(double _x){this->x_est= _x;}
   inline void SetYEst(double _y){this->y_est= _y;}
   inline void SetPreXEst(double _x){this->x_pre= _x;}
   inline void SetPreYEst(double _y){this->y_pre= _y;}
   inline void SetPreZ(double _z){this->z_pre= _z;}
   //
   inline void SetCfgStart( QuadCfg _cfg){this->CfgStart= _cfg; }
   //set all restarts to default
   void SetRestartDefault();
   //to reset the controller
   void ControllerReset();
 
 private:
   //index helps to command the parrot 
   int idx_dubin= -1;//which dubin seg of the path it is in
   int idx_dubin_sub= -1;//which seg of dubin it is in
   //flags
   //for path navigation
   bool if_restart_path= true;//if to start follow a new received path
   bool if_restart_dubin= true;
   bool if_restart_seg= true;
   //for ros communication
   bool if_receive=false;//if receive a dubins curve
   int if_reach =0;//0:not reached, 1:reach time limit, 2:reach target
   //if a new path is received
   bool if_new_path= false;
   //if the flag of if_new_path is received
   bool if_new_rec= false;
   //if it is controlled by joysticks
   bool if_joy= false;
   int uav_state_idx= -1;
   bool lastR1Pressed= false;
   bool lastL1Pressed= false;
   //ros stuffs
   //ros NodeHandle
   ros::NodeHandle nh;
   //publishers
   //planning related
   ros::Publisher pub_rec;//publish if_receive
   ros::Publisher pub_vel;//publish velocity commands
   ros::Publisher pub_reach;//if the quad reach the target,end or time limit
   ros::Publisher pub_state;//the quad publish its state for next cycle planning
   ros::Publisher pub_new_rec;//if we get the new path flag
   ros::Publisher pub_state_idx;//which state the quad is in?
   //quad related
   ros::Publisher takeoff_pub;
   ros::Publisher land_pub;
   ros::Publisher emergency_pub;

   //subscribers
   ros::Subscriber sub_path;//subscribe to generated path
   ros::Subscriber sub_if_new;//subscribe to see if a new path is generated
   ros::Subscriber joy_sub;//subscribe to joystick command
   ros::Subscriber nav_sub;//subscribe to navdateCb

   //services
   ros::ServiceClient flattrim_srv;
   std_srvs::Empty flattrim_srv_srvs;

   //msgs
   //if a dubins curve is received
   std_msgs::Bool rec_msg;
   //if a new path flag is received
   std_msgs::Bool new_rec_msg;
   //if executation of the dubins curve is done
   std_msgs::Int16 reach_msg;
   std_msgs::Int16 state_idx_msg;
   //dubins curve to receive
   ardrone_rrt_avoid::DubinPath_msg path_msg;
   //quad state to publish
   ardrone_rrt_avoid::QuadState_msg state_msg;
   //velocity command to send
   geometry_msgs::Twist twist;
   //path to execute
   std::vector<DubinSeg> dubin_segs;
   //some parameters;
   double v= 1.0;//forward speed
   double vz= 1.0;//rise speed
   double dt= 0.1;//control period
   double yaw_rate= 45./180.*M_PI; 
   double end_r= 1.0*0.5;
   double speed;
   double rho;
   arma::mat K1= 0.1*arma::eye<arma::mat>(3,3); 
   arma::mat K2= 0.1*arma::eye<arma::mat>(3,3);
   //time start the path
   double t_init;
   //for logging trajectory
   std::ofstream log_path;
   std::ofstream log_nav;
   //for controller
   Controller_MidLevelCnt& controlMid;
   //data
   ros::Time tkm1; 
   ros::Time tk;
   ros::Duration elapsed_time;
   
   double elapsed_time_dbl;
   double x_est, y_est, z_mea;
   double x_pre, y_pre, z_pre;
   //in body frame
   double vxm_est , vym_est , yaw_est, vzm_est;
   //in global frome
   double vx_est, vy_est;
   double yawci, vxfi, vyfi, dzfi;
   double pitchco, rollco, dyawco, dzco;
   double YawInit;
   //when t_start
   ros::Time t_start;
   QuadCfg CfgStart;//cfg when takeoff finished
   double x_start,y_start,z_start;
   double d_length;
};

};//end namespace
#endif
