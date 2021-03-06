#include "YlClRRT.hpp"
//std lib
#include <iostream>
#include <fstream>

//user types
#include "UavConfig/ArdroneConfig.h"
#include "UavConfig/BirdConfig.h"
#include "point2D.h"
#include "UavBehavior/ArdroneBehavior.hpp"
#include "UavBehavior/BirdBehavior.hpp"
#include "SpaceLimit.h"
#include "UavState/ArdroneState.h"
#include "UavState/BirdState.h"
#include "ObsCollect.h"
#include "Sampler/SamplerPole.hpp"
//quad related
#include "QuadCfg.h"
#include "quadDubins3D.h"
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
   //double h=19*0.6096,r=2*0.6,l=-2*0.6;
   double h= 40, r=10,l=-10;
   vec_rect.push_back(point2D(0,l) );
   vec_rect.push_back(point2D(h,l) );
   vec_rect.push_back(point2D(h,r) );
   vec_rect.push_back(point2D(0,r) );
   double h_max= 2., h_min= 0.5;
   yc_rrt.SetGeoFence( new SpaceLimit(h_max,h_min,vec_rect) );
   //set the root and the goal
   double x_root=0.,y_root=0.,z_root=0.8,yaw_root=0.;
   double x_goal=14,y_goal=0.6,z_goal=0.8,yaw_goal=0.;
   yc_rrt.SetRoot(new ArdroneState(x_root,y_root,z_root,0.,yaw_root) );
   yc_rrt.SetGoal(new ArdroneState(x_goal,y_goal,z_goal,0.,yaw_goal) );
   //set the uav behavior
   yc_rrt.SetBehavior(new ArdroneBehavior() );
   //set the sampler
   yc_rrt.SetSampler(new SamplerPole() );
   yc_rrt.SetSampleParas();
   //check all the flags
   yc_rrt.CheckFlagsSet();
   //try to sample a node
   //cout<<"test sample node"<< endl;
   yc_rrt.SampleNode();
   //cout<<"tu tu"<< endl;
   //try to execute a dubins curve
   //vector<user_types::obstacle3D > obstacles;
   user_types::ObsCollect obs_collect;
   //obstacles.push_back( user_types::obstacle3D(4,1,M_PI/2,0.,0.8,0.0,0.0,0.5,0.5) );

   QuadCfg start(x_root,y_root,z_root,yaw_root);
   QuadCfg end(x_goal,y_goal,z_goal,yaw_goal);
   quadDubins3D db_3d(start,end,yc_rrt.GetRho() );
   db_3d.OutDubins(0.1);

   double length= 0.;
   vector<user_types::GeneralState*> path_log;
   user_types::GeneralState* st_final= new ArdroneState();
   user_types::GeneralState* st_init= new ArdroneState(x_root+0.0,y_root-0.0,z_root+0.0,0.,yaw_root);
   //cout<<"main accer: "<< st_init->ax<<" "<<st_init->ay<<endl; 
   int coli= utils::DubinsTotalCheck(db_3d,st_init,st_final,end,obs_collect,yc_rrt.GetCheckParasPt(),yc_rrt.GetConfigPt(),yc_rrt.GetSpaceLimitPt(),&path_log,&length);
   
   cout<<"length after: "<< length<<" path_log size: "<< path_log.size()<<" coli: "<< coli<<endl; 

   ofstream myfile("dubin_log.txt");
   
   for(int i=0;i!= path_log.size();++i)
   {
     user_types::GeneralState* st_pt= path_log[i];
     myfile << st_pt->x<<" "<< st_pt->y <<" "<< st_pt->z <<" "<< st_pt->t << endl;
     delete st_pt;
   }//for ends
   
   //path_log.clear();
   bool a;
   double f;
   std::cout<<a <<" "<<f << " "<<path_log.size()<<std::endl;
   myfile.close();
   delete st_final;
   delete st_init;
   //try bird

}//main ends
