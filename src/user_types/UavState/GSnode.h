#pragma once
//tree node for general state
#include "GeneralState.h"

namespace user_types{
  
  struct GSnode{
     GeneralState* state_pt;
     //planning related
     double heuri;//heuristics to the sampled node
     double cost;//cost from root to it
     double cost2go;//cost to the goal
     int idx_dubin;//which dubins curve it belongs to
     int idx_state;//which state in the dubins curve it is
     bool goal_reach;//if accessible from the goal

     Gsnode():state_pt(NULL),heuri(0.),cost(0.),cost2go(0.),idx_dubin(-1),idx_state(-1),goal_reach(false){ };

     Gsnode(GeneralState* _state_pt):state_pt(_state_pt),heuri(0.),cost(0.),cost2go(0.),idx_dubin(-1),idx_state(-1),goal_reach(false){ };
 
  };
};//namespace ends
