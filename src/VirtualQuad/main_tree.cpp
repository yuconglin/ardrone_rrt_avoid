#include "VirtualQuad.hpp"
#include "fstream"
#include "iostream"
#include <gperftools/profiler.h> 
#include "ros/ros.h"
#include "msg.h"
#include "boost/bind.hpp"
#define IF_ROS 0

bool if_receive= false;
bool if_reach= false;
//bool if_send= false;

void receiveCb(const yucong_rrt_avoid::IfSend_msg::ConstPtr& msg)
{
  if_receive= msg->if_receive;
}

void reachCb(const yucong_rrt_avoid::IfReach_msg::ConstPtr& msg)
{
  if_reach= msg->if_reach;
}

void stateCb(const yucong_rrt_avoid::QuadState_msg::ConstPtr& msg,QuadState& st)
{
  st.x= msg->x;
  st.y= msg->y;
  st.z= msg->z;
  st.theta= msg->theta;
  st.v= msg->v;
  st.vz= msg->vz;
  st.t= msg->t;
}


using namespace std;

int main(int argc, char** argv)
{
   VirtualQuad quad;
   quad.SetConfig();
   double speed= quad.GetSpeed();
   //root node
   double x_s = 0;
   double y_s = 0;
   double z_s = 0.5;
   double the_s= 0.;
   double v_s = 0.;
   double vz_s =0.;
   double t_s= 0.;
   quadNode root_node(x_s,y_s,z_s,the_s,v_s,vz_s,t_s);
   //the final goal
   double x_g = x_s+20;
   double y_g = y_s+0.;
   double z_g = z_s+0;
   double the_g= 0.;
   double v_g= 0.;
   double vz_g= 0.;
   double t_g= 0.;
   quadNode goal_final(x_g,y_g,z_g,the_g,v_g,vz_g,t_g);
   //obstacles, we set constant obstacles intialily
   double Dx= goal_final.state.x- root_node.state.x;
   double Dy= goal_final.state.y- root_node.state.y;
   double Dz= goal_final.state.z- root_node.state.z;
   double DM= sqrt(Dx*Dx+Dy*Dy+Dz*Dz);
   double Dgamma= asin(Dz/DM);
   double Dtheta= atan2(Dy,Dx);
   
   double x1=(root_node.state.x+goal_final.state.x)/2;
   double y1=(root_node.state.y+goal_final.state.y)/2;
   double hd1= Dtheta+M_PI;
   double z1=(root_node.state.z+goal_final.state.z)/2;
   double ob_speed= speed*3;
   double v_vert= ob_speed*tan(-Dgamma);
   double t= 0;
   double r= 1. ;
   double del_r = r;
  
   ofstream myfile1("obstacles.txt");
   myfile1<<x1<<" "<<y1<<" "<<hd1<<" "<<ob_speed<<" "<<z1<<" "<<v_vert<<" "<<t<<" "<<r<<endl;
   
   obstacle3D obs1(x1,y1,hd1,ob_speed,z1,v_vert,t,r,del_r);
   vector<obstacle3D> obs_vec(1,obs1);
/*
   double x2=x1;
   double y2=y1+0;
   double z2=z1-0;
   double hd2= M_PI;
   ob_speed= 0.;
   v_vert= 0.;
   myfile1<<x2<<" "<<y2<<" "<<hd2<<" "<<ob_speed<<" "<<z2<<" "<<v_vert<<" "<<t<<" "<<r<<endl;
   
   obstacle3D obs2(x2,y2,hd2,ob_speed,z2,v_vert,t,r,del_r);
   obs_vec.push_back(obs2);

   double x3=x1;
   double y3=y1-3;
   double z3=z1+0;
   double hd3= M_PI;
   ob_speed= 0.;
   v_vert= 0.;
   obstacle3D obs3(x3,y3,hd3,ob_speed,z3,v_vert,t,r,del_r);
   obs_vec.push_back(obs3);

   myfile1<<x3<<" "<<y3<<" "<<hd3<<" "<<ob_speed<<" "<<z3<<" "<<v_vert<<" "<<t<<" "<<r<<endl;
  
   double x4=x1;
   double y4=y1+3;
   double z4=z1-0;
   double hd4= M_PI;
   ob_speed= 0.;
   v_vert= 0.;
   obstacle3D obs4(x4,y4,hd4,ob_speed,z4,v_vert,t,r,del_r);
   myfile1<<x4<<" "<<y4<<" "<<hd4<<" "<<ob_speed<<" "<<z4<<" "<<v_vert<<" "<<t<<" "<<r<<endl;

   obs_vec.push_back(obs4);
   */
   myfile1.close();
   
   quadNode goal_node= goal_final;
   
   quad.TreeSetRoot(root_node);
   quad.TreeSetGoal(goal_node);
   quad.SetSamples3dParas();
   quad.SetObs(obs_vec);
   //test_tree.SetObsRanMove(3,3);
   quad.SetTimeLimit(1.0);

   //ProfilerStart("current.prof");
   //quad.TreeExpand();
   //ProfilerStop();
   cout<<"*******************************************************"<<endl;
   //test_tree.TreeTest();
   QuadState st_init = QuadState(root_node.state.x+0.3,root_node.state.y-0.3,root_node.state.z,0,0,0,0);
   //QuadState st_init= QuadState(root_node.state.x,root_node.state.y,root_node.state.z,0,0,0,0);
   cout<<"if avoid obstacle 1: "<<quad.IfCanAvoid(st_init,obs1)<< endl;
   quad.SetCurrentSt( st_init );
   quad.TreeExpand();
   quad.PathCheckRepeat();
   //output to a file
   vector<QuadState>* real_pt= quad.GetTrajRec();

   ofstream myfile("virtual_path_rec.txt");
   for(int i=0;i!= real_pt->size();++i)
   {
     QuadState st= real_pt->at(i);
     if(myfile.is_open())
       myfile<< st.x <<" "<< st.y<<" "<< st.z<<" "<< st.t << endl;
   }
   myfile.close();
   bool if_new_path= false;
   //******************ROS PART***************************//
#if IF_ROS == 1
   ros::init(argc,argv,"path sender");
   ros::NodeHandle nh;
   //publishers
   ros::Publisher pub_path=nh.advertise<yucong_rrt_avoid::DubinPath_msg>("path",100);
   ros::Publisher pub_if_new=nh.advertise<yucong_rrt_avoid::IfNewPath_msg>("if_new_path",100);
   //subcribers
   ros::Subscriber sub_receive=nh.subscribe("path_rec",100,receiveCb);
   ros::Subscriber sub_reach=nh.subscribe("quad_reach",100,reachCb);
   ros::Subscriber sub_state =nh.subscribe<yucong_rrt_avoid::QuadState_msg>("quad_state",100,boost::bind(stateCb, _1,boost::ref(st_init) ) );
   
   ros::Duration(2.0).sleep();
   quad.SetStartTime( ros::Time::now() );
   //cout<<"time 1: "<<ros::Time::now().toSec()<< endl;
   quad.TreeExpand();
   //cout<<"time 2: "<<ros::Time::now().toSec()<< endl;
   quad.PathCheckRepeat();
   QuadState st_est;
   quad.TimeStateEstimate(1.0, st_est);
   if_new_path= true;
   //it will become false when a path is published
   cout<<"est state: "<<st_est.x<<" "<<st_est.y<<" "<<st_est.z<<" "<<st_est.theta/M_PI*180.<< endl;

   //create path_msg 
   yucong_rrt_avoid::DubinPath_msg path_msg;
   quad.PathToMsg( path_msg );
   yucong_rrt_avoid::IfNewPath_msg if_new_msg;
   //keep sending until received
   while(ros::ok() )
   {
      if(!if_receive)
      {
        pub_path.publish(path_msg);
	//if_new_path= false;
      }
      if_new_msg.if_new_path= if_new_path;
      pub_if_new.publish(if_new_msg);
      //if received,try to plan for the next cycle
      if(if_receive)
      {//emulate planning
        if_receive= false;
	if_new_path= false;
	ros::Duration(1.0).sleep();
      }
      //create path and send
      //planning
      //planning ends,try and get current state
      if(if_reach)
      {
	std::cout<<"yeah,you reach it"<<std::endl;
	if_reach= false;
      }//if_reach ends

      ros::spinOnce();

   }//while ros ends
#endif
 
}//main ends
