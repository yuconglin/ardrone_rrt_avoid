#include "ParrotExe.hpp"
#include "ros/ros.h"
#include "controller/midlevelCnt/Controller_MidLevelCnt.h"
#include "systemtime.h"
#include "std_msgs/Bool.h"
#include "boost/bind.hpp"
#include "QuadCfg.h"
#include "armadillo"

void arriveCb(const std_msgs::Bool::ConstPtr& msg,bool& IfArrive)
{
   IfArrive= msg->data;
}

using namespace Ardrone_rrt_avoid;

int main(int argc, char** argv)
{
   ros::init(argc,argv,"fly_test");

   //the controller
   Controller_MidLevelCnt controlMid;
   //get the system time
   std::string str_time;
   utils::getSystemTime(str_time);
   //config at the start moment
   QuadCfg cfg_start;
   //parameters setup
   int idx_uav_state = -1;
   int pre_uav_state = -1;
   //bool if_reach= false;
   bool if_joy= false;
   bool if_start= false;
   bool if_arrive= false;
   //msgs
   std_msgs::Bool start_msg;

   double c_ux= 0.2, c_uy= 0., c_uz= 0.;
   arma::vec::fixed<3> u_c;
   u_c<< c_ux<< c_uy<< c_uz;

   //file for navdata, and dubin log
   char file_nav[256];
   sprintf( file_nav, "data/%s:%.1f:%.1f:%.1f:%s.txt",str_time.c_str(),c_ux,c_uy,c_uz,"test");
   
   //ParrotExe initialization
   ParrotExe parrot_exe(controlMid,file_nav);
   ros::Duration(1.0).sleep();
   //controller reset
   parrot_exe.ControllerReset();
   //get NodeHandle
   ros::NodeHandle* nh_pt= parrot_exe.GetHandlePt();
   //publisher
   ros::Publisher pub_start =nh_pt->advertise<std_msgs::Bool>("if_start",1);
   ros::Publisher pub_command= nh_pt->advertise<geometry_msgs::Twist>("command",1);
   //subscriber
   ros::Subscriber sub_arrive =nh_pt->subscribe<std_msgs::Bool>("if_arrive",1,boost::bind(arriveCb,_1,boost::ref(if_arrive) ) );
    
   //flat trim
   parrot_exe.sendFlattrim();

   //takeoff
   int takeoff_opt= atoi(argv[1]);
   if(takeoff_opt== 0)
     parrot_exe.sendTakeoff();
   else if(takeoff_opt== 1)
     std::cout<<"manual takeoff please!"<<std::endl;
   else
     std::runtime_error("wrong input. should be 0 or 1");

   geometry_msgs::Twist c_twist;
   /*
   c_twist.linear.x= c_vx;
   c_twist.linear.y= c_vy;
   c_twist.linear.z= c_vz;
   c_twist.angular.z= c_wz;
   */
   //while
   while(ros::ok() )
   {
      idx_uav_state= parrot_exe.GetUavStateIdx();
      if_joy= parrot_exe.GetIfJoy();
      
      //time to send start signal
      if(pre_uav_state== 6 && idx_uav_state== 4 && !if_joy )
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
        //deliver command to fly node
	//pub_command.publish(c_twist);
      }//if ends

      //publish
      start_msg.data= if_start;
      pub_start.publish(start_msg);
      //deliver command to fly node
      //it is based on current velocity
      parrot_exe.StepResponse( u_c, c_twist );
      pub_command.publish(c_twist);

      pre_uav_state= idx_uav_state;
      
      if(idx_uav_state== 4 && if_arrive && !if_joy)
	parrot_exe.sendLand();
      
      ros::spinOnce();
   }//while ends

}

