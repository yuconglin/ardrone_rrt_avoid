#include "YlClRRT.hpp"
//std lib
#include <iostream>
#include <fstream>
//user types
#include "UavConfig/BirdConfig.h"
#include "point2D.h"
#include "UavBehavior/BirdBehavior.hpp"
#include "SpaceLimit.h"
#include "UavState/BirdState.h"
#include "obstacle3D.h"
#include "Sampler/SamplerPole.hpp"

using namespace std;
using namespace Bird_rrt_avoid;
using namespace user_types;

int main(int argc, char** argv)
{
    YlClRRT yc_rrt;
    //set config
    yc_rrt.SetConfig(new BirdConfig() );
    yc_rrt.ConfigFill("/home/yucong/fuerte_workspace/sandbox/ardrone_rrt_avoid/params/bird_params.xml");
    yc_rrt.SetCheckParas();
    //set geo fence
    vector<point2D> vec_rect;
    double scale= 70;
    double h=40*scale,r=10*scale,l=-10*scale;
    vec_rect.push_back(point2D(0,l) );
    vec_rect.push_back(point2D(h,l) );
    vec_rect.push_back(point2D(h,r) );
    vec_rect.push_back(point2D(0,r) );
    double h_max= 2*scale, h_min= 0.5*scale;
    yc_rrt.SetGeoFence( new SpaceLimit(h_max,h_min,vec_rect) );
    //set the root and the goal
    double x_root=0.,y_root=0,z_root=0.8*scale,yaw_root=0.;
    double x_goal=14*scale,y_goal=0.6*scale,z_goal=0.8*scale,yaw_goal=0.;
    yc_rrt.SetGoal(new BirdState(x_goal,y_goal,z_goal,0.,yaw_goal) );
    //set the uav behavior
    yc_rrt.SetBehavior(new BirdBehavior() );
    //set the sampler
    yc_rrt.SetSampler(new SamplerPole() );
    //set the obstacle
    vector<obstacle3D> obs3d;
    obs3d.push_back( obstacle3D(7*scale,0,0.7*scale,-1*scale,0,0,0,0.5*scale,0.5*scale) );
    yc_rrt.SetObs3D(obs3d);
    //set parameters for tree expand
    yc_rrt.SetTimeLimit(1.0);
    yc_rrt.SetIfInRos(false);
    
    yc_rrt.ClearToDefault();
    yc_rrt.ClearTree();
    yc_rrt.SetRoot(new ArdroneState(x_root,y_root,z_root,0,0) );
    //expand tree
    yc_rrt.SetSampleParas();
    yc_rrt.ExpandTree();
    ArdroneState* st_current= new ArdroneState(x_root,y_root,z_root,0,0);
    yc_rrt.PathCheckRepeat(st_current);
    delete st_current;
    //clear the tree
    yc_rrt.ClearTree();
    vector<GeneralState*>* traj_pt= yc_rrt.GetTrajRecPt();
    ofstream myfile("traj_bird.txt");
    for(int i=0;i!= traj_pt->size();++i)
    {
       GeneralState* st= traj_pt->at(i);
       myfile<< st->x<<" "<< st->y<<" "<< st->z<<" "<< st->t << endl;
       delete st;
    }//for ends
    myfile.close();
}//main ends
