#pragma once
//generic tree 
#include "tree.hh"
//uav state
#include "UavState/GeneralState.h"
//tree node
#include "UavState/GSnode.h"
//used structs
//not everyone needs to be an object, some can just be pointers
//list for cpp funcions to include
#include "Sampler3D/Sampler3D.hpp"
//user types
#include "UavBehavior/GeneralBehavior.hpp"
#include "UavConfig/GeneralConfig.h"
#include "UavState/GeneralState.h"
#include "SpaceLimit.h"

typedef tree<user_types::GSnode>::iterator TREEIter;

namespace Ardrone_rrt_avoid{

   class YlClRRT{
     
     public:
       //about config
       YlClRRT();
       //~YlClRRT();
       //config
       inline void SetConfig(user_types::GeneralConfig* _config_pt){this->config_pt= _config_pt;}
       void ConfigFill(const char* pFilename); 
       //about set dubins collision parameters
       void SetCheckParas();
       //about geo-fencing
       void SetGeoFence(user_types::SpaceLimit* _space_pt);
       //access
       inline double GetRho(){return config_pt->rho;}
       inline user_types::checkParas* GetCheckParasPt(){return &checkParas;}
       inline user_types::GeneralConfig* GetConfigPt(){return config_pt;}
       //set root and goal node
       void SetRoot( user_types::GeneralState* state_pt );
       void SetGoal( user_types::GeneralState* state_pt );
       void SetBehavior(user_types::GeneralBehavior* _behavior_pt );
       //about sample
       void SetSampleParas();
       void SampleNode();
       //check all necesarry flags
       void CheckFlagsSet();
       //expand trees
       void ExpandTree();
     private:
       //user defined times
       user_types::Sampler3D sampler;
       user_types::SpaceLimit* spaceLimit_pt;
       user_types::GeneralConfig* config_pt;
       user_types::GeneralBehavior* behavior_pt;
       user_types::checkParas checkparas;
       //root and goal nodes
       user_types::GSnode goal_node;
       user_types::GSnode root_node;
       //sampled node
       user_types::GSnode sample_node;
       //protection flags
       bool if_goal_set;
       bool if_root_set;
       bool if_goal_reach;
       bool if_sampler_para_set;
       bool if_config_set;
       bool if_spacelimit_set;
       bool if_behavior_set;
       bool if_checkparas_set;
       //check for flags
       void CheckGoalSet();
       void CheckRootSet();
       void CheckGoalReach();
       void CheckSampleParaSet();
       void CheckConfigSet();
       void CheckSpaceLimitSet();
       void CheckBehaviorSet();
       void CheckParasSet();

   };//class YlClRRT ends


}//namespace ends
