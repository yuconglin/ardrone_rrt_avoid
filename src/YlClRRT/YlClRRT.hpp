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
//#include "SpaceLimit.h"

using namespace user_types;

typedef tree<GSnode>::iterator TREEIter;

namespace Ardrone_rrt_avoid{

   class YlClRRT{
     
     public:
       void SetSampleParas();
       void SampleNode();
     private:
       //user defined times
       user_types::Sampler3D sampler;
       user_types::SpaceLimit* spaceLimit_pt;
       user_types::GeneralConfig* config_pt;
       user_types::GeneralBehavior* behavior_pt;
       //root and goal nodes
       GSnode goal_node;
       GSnode root_node;
       //sampled node
       GSnode sample_node;
       //protection flags
       bool if_goal_set;
       bool if_root_set;
       bool if_goal_reach;
       bool if_sampler_para_set;
       bool if_config_set;
       bool if_spacelimit_set;
       //check for flags
       void CheckGoalSet();
       void CheckRootSet();
       void CheckGoalReach();
       void CheckSampleParaSet();
       void CheckConfigSet();
       void CheckSpaceLimitSet();

   }//class YlClRRT ends


}//namespace ends
