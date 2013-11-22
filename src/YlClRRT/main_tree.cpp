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
#include "obstacle3D.h"
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
    vec_rect.push_back(point2D(0,-3) );
    vec_rect.push_back(point2D(10,-3) );
    vec_rect.push_back(point2D(10,3) );
    vec_rect.push_back(point2D(0,3) );
    double h_max= 2., h_min= 0.5;
    yc_rrt.SetGeoFence( new SpaceLimit(h_max,h_min,vec_rect) );
    //set the root and the goal
    double x_root=0.,y_root=0.,z_root=0.8,yaw_root=0.;
    double x_goal=8.,y_goal=0.,z_goal=0.8,yaw_goal=0.;
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
    vector<obstacle3D> obstacles;
    obstacles.push_back(obstacle3D(4.,0.,0.,0.,0.8,0.,0.,0.5,0.5) );
    yc_rrt.SetObs(obstacles);
    //set parameters for tree expand
    yc_rrt.SetTimeLimit(1.0);
    yc_rrt.SetIfInRos(false);
    yc_rrt.ExpandTree();
}//main ends
