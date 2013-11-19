#include "YlClRRT.hpp"
//user types
#include "UavConfig/ArdroneConfig.h"
#include "point2D.h"
#include "UavBehavior/ArdroneBehavior.hpp"
#include "SpaceLimit.h"
#include "UavState/ArdroneState.h"
#include "obstacle3D.h"
//quad related
#include "QuadCfg.h"
#include "quadDubins3D.h"
//std lib
#include <iostream>
#include <fstream>
//utils
#include "DubinsTotalCheck.h" 

using namespace std;
using namespace Ardrone_rrt_avoid;
using namespace user_types;

int main(int argc, char** argv)
{
   YlClRRT yc_rrt;
   //set config
   yc_rrt.SetConfig( new ArdroneConfig() );
   yc_rrt.ConfigFill("/home/yucong/.ros/param.xml");
   yc_rrt.SetCheckParas();
   //seg geo fence
   vector<point2D> vec_rect;
   vec_rect.push_back(point2D(0,-3) );
   vec_rect.push_back(point2D(10,-3) );
   vec_rect.push_back(point2D(10,3) );
   vec_rect.push_back(point2D(0,3) );
   double h_max= 2., h_min= 0.5;
   yc_rrt.SetGeoFence( new SpaceLimit(h_max,h_min,vec_rect) );
   //set the root and the goal
   double x_root=0.,y_root=0.,z_root=0.8,yaw_root=0.;
   double x_goal=8.,y_goal=4.,z_goal=0.8,yaw_goal=0.;
   yc_rrt.SetRoot(new ArdroneState(x_root,y_root,z_root,0.,yaw_root) );
   yc_rrt.SetGoal(new ArdroneState(x_goal,y_goal,z_goal,0.,yaw_goal) );
   //set the uav behavior
   yc_rrt.SetBehavior(new ArdroneBehavior() );
   //set the sampler
   yc_rrt.SetSampleParas();
   //check all the flags
   yc_rrt.CheckFlagsSet();
   //try to sample a node
   cout<<"test sample node"<< endl;
   yc_rrt.SampleNode();
   //try to execute a dubins curve
   vector<obstacle3D> obstacles;
   QuadCfg start(x_root,y_root,z_root,yaw_root);
   QuadCfg end(x_goal,y_goal,z_goal,yaw_goal);
   quadDubins3D db_3d(start,end,yc_rrt.GetRho() );
   db_3d.OutDubins(0.1);

   double length= 0.;
   vector<user_types::GeneralState*> path_log;
   user_types::GeneralState* st_final= NULL;
   user_types::GeneralState* st_init= new ArdroneState(x_root+0.4,y_root-0.4,z_root+0.2,yaw_root);
   utils::DubinsTotalCheck(db_3d,st_init,st_final,end,obstacles,*yc_rrt.GetCheckParaPt(),
			 yc_rrt.GetConfigPt(),path_log,length);
   cout<<"length after: "<< length<< endl;
   
   ofstream myfile("dubin_log.txt");
   for(int i=0;i!= path_log.size();++i)
   {
     user_types::GeneralState* st_pt= path_log[i];
     myfile << st_pt->x<<" "<< st_pt->y <<" "<< st_pt->z <<" "<< st_pt->t << endl;
     delete st_pt;
   }//for ends
   myfile.close();
 
}//main ends
