#pragma once
//generic tree 
#include "tree.hh"
//user types
//#include "obstacle3D.h"
#include "ObsCollect.h"
//tree node
#include "UavState/GSnode.h"
//ros
#include "ros/ros.h"
//quad related
#include "quadDubins3D.h"

//forward declaration
namespace user_types{
   struct GeneralState;
   class Sampler3D;
   struct GeneralBehavior;
   struct GeneralConfig;
   struct SpaceLimit;
   struct checkParas;
}

typedef tree<user_types::GSnode>::iterator TREEIter;

namespace Ardrone_rrt_avoid{

   class YlClRRT{
     
     public:
       //about config
       YlClRRT();
       ~YlClRRT();
       //config
       inline void SetConfig(user_types::GeneralConfig* _config_pt){this->config_pt= _config_pt;}
       void ConfigFill(const char* pFilename); 
       //about set dubins collision parameters
       void SetCheckParas();
        //set user-defined types
       void SetBehavior(user_types::GeneralBehavior* _behavior_pt );
       void SetSampler(user_types::Sampler3D* _sampler_pt );
       //about geo-fencing
       void SetGeoFence(user_types::SpaceLimit* _space_pt);
       //access
       double GetRho();
       inline user_types::checkParas* GetCheckParasPt(){return this->checkparas_pt;}
       inline user_types::GeneralConfig* GetConfigPt(){return config_pt;}
       inline std::vector<user_types::GeneralState*>* GetTrajRecPt(){return &traj_rec;}
       //set root and goal node
       void SetRoot( user_types::GeneralState* state_pt );
       void SetGoal( user_types::GeneralState* state_pt );
       //about sample
       void SetSampleParas();
       void SampleNode();
       //set obstacles
       void SetObs2D(const std::vector<user_types::obstacle2D>& _obs2ds);
       void SetObs3D(const std::vector<user_types::obstacle3D>& _obs3ds);
       //check all necesarry flags
       void CheckFlagsSet();
       //time related
       inline void SetTimeLimit(double _limit){t_limit= _limit;}
       inline void SetIfInRos(bool _if_in_ros){if_in_ros= _if_in_ros;}
       inline void SetStartTime(ros::Time _t_start){t_start= _t_start;}
       //expand trees
       void ExpandTree();
       //clean the tree
       void ClearTree();
       void ClearSubTree(TREEIter sub_it);
       //generate the path
       bool PathGen();
       //PathCheck
       bool PathCheck(user_types::GeneralState* st_init,int& it_idx,std::vector<user_types::GeneralState*>& traj_rec);
       bool PathCheckRepeat(user_types::GeneralState* st_current);
     
     private:
       //path log
       std::vector<user_types::GeneralState*> temp_log;
       std::vector<user_types::GeneralState*> traj_rec;
       //the tree body
       tree<user_types::GSnode> main_tree;
       std::vector<TREEIter> tree_vector;
       std::vector<TREEIter> tree_vector_sort;
       std::vector<TREEIter> goal_connect_nodes;
       std::vector<quadDubins3D> dubin_collects;
       //path generated
       std::vector<TREEIter> path_total;
       //obstacles
       //std::vector<user_types::obstacle3D> obstacles;
       user_types::ObsCollect obs_collect; 
       //user defined types
       user_types::Sampler3D* sampler_pt;
       user_types::SpaceLimit* spaceLimit_pt;
       user_types::GeneralConfig* config_pt;
       user_types::GeneralBehavior* behavior_pt;
       user_types::checkParas* checkparas_pt;
       //root and goal nodes
       user_types::GSnode goal_node;
       user_types::GSnode root_node;
       //sampled node
       user_types::GSnode sample_node;
       //current state
       user_types::GeneralState* st_current;
       //protection flags
       bool if_goal_set;
       bool if_root_set;
       bool if_goal_reach;
       bool if_sampler_para_set;
       bool if_config_set;
       bool if_spacelimit_set;
       bool if_behavior_set;
       bool if_checkparas_set;
       //time related
       double t_limit;
       ros::Time t_start;
       bool if_limit_reach;
       bool if_in_ros;
       double sec_count;
       //private function
       bool CheckGoalReach( TREEIter it); 
       double Heuristics(user_types::GSnode& node);
       void CalHeuri();
       void SortNodes();
       void InsertDubinsNode(TREEIter it);
       void TempLogClear();
       //check for flags
       void CheckGoalSet();
       void CheckRootSet();
       void CheckGoalIfReach();
       void CheckSampleParaSet();
       void CheckConfigSet();
       void CheckSpaceLimitSet();
       void CheckBehaviorSet();
       void CheckParasSet();

   };//class YlClRRT ends


}//namespace ends
