#include "ParrotPlan.hpp"
#include "YlClRRT.hpp"
//std lib
#include <iostream>
#include <fstream>
//user types
#include "UavConfig/ArdroneConfig.h"
#include "point2D.h"
#include "UavBehavior/ArdroneBehavior.hpp"
#include "SpaceLimit.h"
#include "UavState/ArdroneState.h"
#include "obstacle2D.h"
#include "Sampler3D/Sampler3D.hpp"

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
    double h=19*0.6096,r=2*0.6,l=-2*0.6;
    vec_rect.push_back(point2D(0,l) );
    vec_rect.push_back(point2D(h,l) );
    vec_rect.push_back(point2D(h,r) );
    vec_rect.push_back(point2D(0,r) );
    double h_max= 2., h_min= 0.5;
    yc_rrt.SetGeoFence( new SpaceLimit(h_max,h_min,vec_rect) );
    //set the root and the goal
    double x_root=0.,y_root=0.,z_root=0.8,yaw_root=0.;
    double x_goal=11.,y_goal=0.,z_goal=0.8,yaw_goal=0.;
    yc_rrt.SetRoot(new ArdroneState(x_root,y_root,z_root,0.,yaw_root) );
    yc_rrt.SetGoal(new ArdroneState(x_goal,y_goal,z_goal,0.,yaw_goal) );
    //set the uav behavior
    yc_rrt.SetBehavior(new ArdroneBehavior() );
    //set the sampler
    yc_rrt.SetSampler(new Sampler3D() );
    yc_rrt.SetSampleParas();
    //check all the flags
    yc_rrt.CheckFlagsSet();
    //set the obstacles
    vector<obstacle2D> obs2d;
    obs2d.push_back( obstacle2D(7.3152,-0.2032,0.3,0.3) );
    //write the 2D obstacles to a text file
    ofstream obs2d_file("obs2ds.txt");
    for(int i=0;i!=obs2d.size();++i)
    {
      obs2d_file<<obs2d[i].x<<" "<<obs2d[i].y<<" "<<obs2d[i].r<<" "<<obs2d[i].delta_r<< endl;
    }//for ends
    obs2d_file.close();
    //
    yc_rrt.SetObs2D( obs2d );
    //set parameters for tree expand
    yc_rrt.SetTimeLimit(1.0);
    yc_rrt.SetIfInRos(true);
    //ros start
    ros::init(argc,argv,"planner");
    //initialize the object
    ParrotPlan planner(&yc_rrt);
    planner.SetTOffset(1.0); 
    planner.working();    
    return 0; 
    /*
    yc_rrt.ExpandTree();
    ArdroneState* st_current= new ArdroneState(x_root+0.4,y_root-0.4,z_root+0.1,0,0);
    yc_rrt.PathCheckRepeat(st_current);
    delete st_current;
    //yc_rrt.PathGen();
    //clear the tree
    yc_rrt.ClearTree();
    vector<GeneralState*>* traj_pt= yc_rrt.GetTrajRecPt();
    ofstream myfile("traj_rec.txt");
    for(int i=0;i!= traj_pt->size();++i)
    {
       GeneralState* st= traj_pt->at(i);
       myfile<< st->x<<" "<< st->y<<" "<< st->z<<" "<< st->t << endl;
       delete st;
    }//for ends
    myfile.close();
    */
}//main ends
