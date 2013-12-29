#include "ParrotPlan.hpp"
#include "YlClRRT/YlClRRT.hpp"
//std lib
#include <iostream>
#include <fstream>
//user types
#include "systemtime.h"
#include "UavConfig/ArdroneConfig.h"
#include "point2D.h"
#include "UavBehavior/ArdroneBehavior.hpp"
#include "SpaceLimit.h"
#include "UavState/ArdroneState.h"
#include "obstacle2D.h"
//sampler
#include "Sampler/SamplerPole.hpp"
#include "Sampler/SamplerRect.hpp"
//updater
#include "ObsUpdater/ObsUpdaterReal.hpp"
#include "ObsUpdater/ObsUpdaterVirtual.hpp"

using namespace std;
using namespace Ardrone_rrt_avoid;
using namespace user_types;
//change, spaceLimit width, sampling width
int main(int argc, char** argv)
{
    YlClRRT yc_rrt;
    //set config
    yc_rrt.SetConfig( new ArdroneConfig() );
    yc_rrt.ConfigFill("/home/yucong/.ros/param.xml");
    yc_rrt.SetCheckParas();
    //seg geo fence
    vector<point2D> vec_rect;
    double h=400,r=100,l=-100;
    //double h= 30,r= 2, l= -2;
    vec_rect.push_back(point2D(-h,l) );
    vec_rect.push_back(point2D(h,l) );
    vec_rect.push_back(point2D(h,r) );
    vec_rect.push_back(point2D(-h,r) );
    double h_max= 2., h_min= 0.5;
    yc_rrt.SetGeoFence( new SpaceLimit(h_max,h_min,vec_rect) );
    //set the root and the goal
    double e= 0.6096; 
    double x_root=0,y_root=0,z_root=0.7,yaw_root=0.;
    double x_goal=15,y_goal=0,z_goal=0.8,yaw_goal=0.;
    //double x_goal= 8., y_goal=0.,z_goal=0.8,yaw_goal=0.; 
    yc_rrt.SetRoot(new ArdroneState(x_root,y_root,z_root,0.,yaw_root) );
    yc_rrt.SetGoal(new ArdroneState(x_goal,y_goal,z_goal,0.,yaw_goal) );
    //set the uav behavior
    yc_rrt.SetBehavior(new ArdroneBehavior() );
    //set the sampler
    yc_rrt.SetSampler(new SamplerPole() );
    //yc_rrt.SetSampler(new SamplerRect() );
    yc_rrt.SetSampleParas();
    //check all the flags
    yc_rrt.CheckFlagsSet();
    //set the obstacles
    vector<obstacle2D> obs2d;
    obs2d.push_back( obstacle2D(7.3152,-0.2032,0.6,0.6) );
    //write the 2D obstacles to a text file
    ofstream obs2d_file("obs2ds.txt");
    for(int i=0;i!=obs2d.size();++i)
    {
      obs2d_file<<obs2d[i].x<<" "<<obs2d[i].y<<" "<<obs2d[i].r<<" "<<obs2d[i].delta_r<< endl;
    }//for ends
    obs2d_file.close();
    //
    //yc_rrt.SetObs2D( obs2d );
    //set parameters for tree expand
    yc_rrt.SetTimeLimit(1.0);
    yc_rrt.SetIfInRos(true);
     
    //ros start
    ros::init(argc,argv,"planner");
    //get the system time
    std::string str_time;
    utils::getSystemTime(str_time);
    //set log file name
    char file_log[256];
    //sprintf( file_log, "data/%s:%s.txt",str_time.c_str(),"plan");
    sprintf(file_log,"/home/yucong/ros_workspace/ardrone_rrt_avoid/data/%s",str_time.c_str());
    //initialize the object
    ParrotPlan planner(&yc_rrt,file_log);
    //planner.SetObsUpdater(new ObsUpdaterReal(0,1) );
      
    std::vector<std::string> filenames;
    filenames.push_back(std::string("/home/yucong/ros_workspace/ardrone_rrt_avoid/record_4.txt") );
    planner.SetObsUpdater(new ObsUpdaterVirtual(filenames,1.,0.5) );
    
    planner.SetTOffset(1.0); 
    planner.working();    
    //planner.PathPlanning();
    return 0; 
}//main ends
