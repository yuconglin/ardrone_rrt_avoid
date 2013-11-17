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
       inline void SetConfig(user_types::GeneralConfig* _config_pt){this->config_pt= _config_pt;}
       void ConfigFill(const char* pFilename);
       
       //about geo-fencing
       void SetGeoFence(user_types::SpaceLimit* _space_pt);

       //set root and goal node
       void SetRoot( user_types::GeneralState* state_pt );
       void SetGoal( user_types::GeneralState* state_pt );
       void SetBehavior(user_types::GeneralBehavior* _behavior_pt );
 
       void SetSampleParas();
       void SampleNode();

       void CheckFlagsSet();

     private:
       //user defined times
       user_types::Sampler3D sampler;
       user_types::SpaceLimit* spaceLimit_pt;
       user_types::GeneralConfig* config_pt;
       user_types::GeneralBehavior* behavior_pt;
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
       //check for flags
       void CheckGoalSet();
       void CheckRootSet();
       void CheckGoalReach();
       void CheckSampleParaSet();
       void CheckConfigSet();
       void CheckSpaceLimitSet();
       void CheckBehaviorSet();
       
   };//class YlClRRT ends


}//namespace ends
