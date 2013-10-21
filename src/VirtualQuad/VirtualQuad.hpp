#pragma once
#include "QuadState.h"
#include "quadDubins3D.h"
#include "QuadCfg.h"
#include "quadNode.h"
#include "QuadConfig.hpp"
#include "armadillo"
#include "common_struct.hpp"
#include "msg.h"
#include "tree.hh"

typedef tree<quadNode>::iterator TREEIter;
//it can actually be seperated into two classes plus at least two free functions
class VirtualQuad {
   public:
     //BASIC
     void SetConfig(double _v=1,double _w=45./180*M_PI,double _vz=1.,double _dt=0.1);
     void StateUpdate(const QuadState& st,const arma::colvec& u,QuadState& st_next,double dt);
     inline double GetRho(){return this->rho;}
     inline double GetSpeed(){return this->speed;}
     inline vector<QuadState>* GetPathLog(){return &(this->path_log);}
     inline vector<QuadState>* GetTrajRec(){return &(this->traj_rec);}
     inline double GetDt(){return this->config.dt;}
     int ParamFromXML(const char* pFilename="/home/yucong/fuerte_workspace/sandbox/yucong_rrt_avoid/src/common/param.xml");
     //int ParamFromXML(const char* pFilename="/home/yucong/ros_workspace/yucong_rrt_avoid/src/common/param.xml");
     //DUBINS
     int DubinsTotalCheck(double t_sample,quadDubins3D& db_3d,const QuadState& st_init,const QuadCfg& cfg_target,QuadState& st_next,QuadState& st_sample,const vector<obstacle3D>& obstacles,int idx_seg= -1);    

     int DubinsSubCheck(quadDubins3D& db_3d,const QuadState& st_init,const QuadCfg& cfg_target,int idx_seg,QuadState& st_next,const vector<obstacle3D>& obstacles);
     
     void DubinToMsg(const quadDubins3D& db_3d,const QuadCfg& cfg_target,yucong_rrt_avoid::DubinSeg_msg& db_msg);
     //RRT TREE
     //current state to check path
     inline void SetCurrentSt(const QuadState& _st_current) {this->st_current= _st_current;}
     //set time limit for tree expanding
     inline void SetTimeLimit(const double _t_limit) {this->t_limit=_t_limit;}
     inline double GetTimeLimit(){return t_limit;}

     void TreeSetRoot(const quadNode& root_nd);
     void TreeSetGoal(const quadNode& goal_nd);
     quadNode GetRoot(){return this->root_node;}
     quadNode GetGoal(){return this->goal_node;}

     void SetSamples3dParas();
     void SamplingNodes();
     void SamplingNodesNew();
     bool CheckSample();
     double Heuristics( quadNode& node );
     void CalHeuri();
     void SortNodes( );
     void SetObs(const vector<obstacle3D> obses);
     bool CheckGoalReach(TREEIter it);
     void InsertDubinsNode(TREEIter start_it);
     void TreeExpand();
     bool PathGen();
     bool PathCheck(QuadState& st_init,TREEIter& it_block,vector<QuadState>& traj_rec);//true if collided
     bool PathCheckRepeat();
     void TimeStateEstimate(double Dt, QuadState& st);
     void PathToMsg(yucong_rrt_avoid::DubinPath_msg& path_msg);
     void SetObsRanMove(int sta_num, int dy_num);
     //set to default
     void ClearToDefault();
     void TreeClear();
     //set start time
     inline void SetStartTime(ros::Time _t_start){this->t_start=_t_start; }
     //avoid feasibility check
     bool IfCanAvoid(const QuadState& st_current,const obstacle3D& obs); 
     //set in ros
     inline void SetInRos(){ this->if_in_ros=true;}

protected:
     QuadConfig config;
     //quadConfig config;
     double rho;//constant turn radius
     double speed;
     //for dubins curve
     double ds_check;//for collision check
     QuadState st_now; //virtual current state,primarily used in planning
     arma::mat K1= 10*arma::eye<arma::mat>(3,3); 
     arma::mat K2= 10*arma::eye<arma::mat>(3,3);
     double dubin_actual_length= 0.;
     double end_r;
     vector<QuadState> path_log;
     //for rrt tree
     QuadState st_current;
     vector<obstacle3D> obstacles;
     double ds_insert;//interval to insert nodes
     tree<quadNode> main_tree;//tree body
     vector<TREEIter> tree_vector;//a vector of all tree nodes
     vector<TREEIter> tree_vector_sort;//vector of sorted nodes
     vector<TREEIter> goal_connect_nodes;//vector of nodes connected to the goal
     vector<quadDubins3D> dubin_collects;
     //executed states log
     vector<QuadState> traj_rec;
     SampleParas3D my_sampleparas;
     bool if_goal= false;
     bool if_goal_set= false;
     bool if_samplepara_set= false;
     quadNode sample_node;
     quadNode root_node;
     quadNode goal_node;
     //path generated
     vector<TREEIter> path_total;
     //for timing
     double t_limit= 1.0;
     ros::Time t_start;
     double sec_count=0.;
     bool if_limit_reach= false;
     //for sampling limit
     double H_MAX= 100;
     double H_MIN= 0.3;
     //flag indicate if in ros
     bool if_in_ros= false;
};//Class ends
