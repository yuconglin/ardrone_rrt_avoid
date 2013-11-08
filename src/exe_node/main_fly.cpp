#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Twist.h"
#include "boost/bind.hpp"

void startCb(const std_msgs::Bool::ConstPtr& msg,bool& IfStart);

int main(int argc, char** argv)
{//just fly. send command and execute
   //boolean variables to indicate if start or land
   bool if_start= false;
   bool if_arrive= false;
   bool if_fly= false;
   //ros init
   ros::init(argc,argv,"fly");
   //basically, we received a singnal, start, when done, send a singal to another node
   //so it can land
   ros::NodeHandle nh;
   //subscribers
   ros::Subscriber sub_start =nh.subscribe<std_msgs::Bool>("if_start",1,boost::bind(startCb,_1,boost::ref(if_start) ) );
   //publishers
   ros::Publisher pub_twist =nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
   ros::Publisher pub_arrive =nh.advertise<std_msgs::Bool>("if_arrive", 1); 
   //command to send
   geometry_msgs::Twist twist;
   twist.linear.x= 0.5;
   twist.linear.y= 0.;
   twist.linear.z= 0.;
   twist.angular.z= 0.;
   //msgs
   std_msgs::Bool arrive_msg;
   double dur= 1.0;
   //while
   while(ros::ok() )
   {
     if(if_start && !if_fly){
       if_fly= true;
       pub_twist.publish(twist);
       ros::Duration(dur).sleep();
       if_arrive= true;

     }//if_start ends

     if(if_arrive)
     {
       pub_twist.publish( geometry_msgs::Twist() );
       ros::Duration(0.5).sleep();
     }
     
     arrive_msg.data= if_arrive;
     pub_arrive.publish(arrive_msg);

     ros::spinOnce();
   }//while ends

}//main ends

void startCb(const std_msgs::Bool::ConstPtr& msg,bool& IfStart)
{
   IfStart= msg->data; 
}//startCb ends
