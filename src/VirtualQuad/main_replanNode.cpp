#include "ReplanNode.hpp"
#include "VirtualQuad.hpp"
#include <iostream>

using namespace std;

int main(int argc, char** argv)
{
  VirtualQuad quad;
  //set quad config
  quad.SetConfig(); 
  double t_offset=1.0;
  //set root node
  double x_s=0,y_s=0,z_s=0.5,the_s=0.,v_s=0,vz_s=0.,t_s=t_offset;
  quadNode root_node(x_s,y_s,z_s,the_s,v_s,vz_s,t_s);
  quad.TreeSetRoot(root_node);
  //set goal node
  double x_g = x_s+20;
  double y_g = y_s+0.;
  double z_g = z_s+0;
  double the_g= 0.;
  double v_g= 0.;
  double vz_g= 0.;
  double t_g= 0.;
  quadNode goal_final(x_g,y_g,z_g,the_g,v_g,vz_g,t_g);
  quad.TreeSetGoal(goal_final);
  quad.SetSamples3dParas();
  quad.SetTimeLimit(1.0);
  //choose which method to set obstacles
  int input = 0;
  /*
  while(input!=0&&input!=1)
  {
    cout<< "please choose which way you want the obstacle: 0.normal obstacles,1.quad obstacle" << endl;
    cin >> input;
    if(input!=0&&input!=1)
      cout<<"wrong input,please type again."<<endl;
  }*/
  //ros starts
  ros::init(argc,argv,"replanning");
  //ReplanNode
  ReplanNode replan_nd(quad,input);
  replan_nd.SetTOffset(1.0);
  replan_nd.working();
  return 0;
}//main ends
