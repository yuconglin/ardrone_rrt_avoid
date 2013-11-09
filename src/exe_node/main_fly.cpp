#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Twist.h"
#include "boost/bind.hpp"

void startCb(const std_msgs::Bool::ConstPtr& msg,bool& IfStart);
void cmdCb(const geometry_msgs::Twist::ConstPtr& msg, geometry_msgs::Twist& c_twist);

int main(int argc, char** argv)
{//just fly. send command and execute
   //boolean variables to indicate if start or land
   bool if_start= false;
   bool if_arrive= false;
   bool if_fly= false;
   //command to send
   geometry_msgs::Twist c_twist= geometry_msgs::Twist();
   geometry_msgs::Twist pre_twist= c_twist;
   //ros init
   ros::init(argc,argv,"fly");
   //basically, we received a singnal, start, when done, send a singal to another node
   //so it can land
   ros::NodeHandle nh;
   //subscribers
   ros::Subscriber sub_start =nh.subscribe<std_msgs::Bool>("if_start",1,boost::bind(startCb,_1,boost::ref(if_start) ) );
   ros::Subscriber sub_command= nh.subscribe<geometry_msgs::Twist>("command",1,boost::bind(cmdCb,_1,boost::ref(c_twist) ) );
   
   //publishers
   ros::Publisher pub_twist =nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
   ros::Publisher pub_arrive =nh.advertise<std_msgs::Bool>("if_arrive", 1); 
   //msgs
   std_msgs::Bool arrive_msg;
   double dur= 3.0, dt= 0.1;
   //while
   while(ros::ok() )
   {
     //if(if_start && !if_fly)
     if(if_start)
     {
       //if_fly= true;
       if( c_twist.linear.x!= pre_twist.linear.x
         ||c_twist.linear.y!= pre_twist.linear.y
	 ||c_twist.linear.z!= pre_twist.linear.z
	 ||c_twist.angular.z!= pre_twist.angular.z 
	 &&c_twist.linear.x>0||c_twist.linear.y>0
	 ||c_twist.linear.z>0
	 )
       {
	 std::cout<<"tuba tuba"<<std::endl;
	 std::cout<< c_twist.linear.x<<" "
		  << c_twist.linear.y<<" "
		  << c_twist.linear.z<<" "
                  << c_twist.angular.z<< std::endl;

	 pub_twist.publish(c_twist);
	 pre_twist= c_twist;
         ros::Duration(dt).sleep();
         dur-= dt;
	 //if_arrive= true;
       }
     }//if_start ends

     if(dur<= 0.) if_arrive= true;

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

void cmdCb(const geometry_msgs::Twist::ConstPtr& msg, geometry_msgs::Twist& c_twist)
{
   c_twist= *msg;
}//cmdCb ends
