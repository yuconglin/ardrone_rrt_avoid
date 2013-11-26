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
     //int idx_state;//which state in the dubins curve it is
     double idx_length;//length from the start of the dubin's curve
     bool goal_reach;//if accessible from the goal

     GSnode():state_pt(NULL),heuri(0.),cost(0.),cost2go(0.),idx_dubin(-1),idx_length(0.),goal_reach(false){ };

     GSnode(GeneralState* _state_pt):state_pt(_state_pt->copy()),heuri(0.),cost(0.),cost2go(0.),idx_dubin(-1),idx_length(0.),goal_reach(false){ };

     GSnode(GeneralState* _state_pt,double _heuri,double _cost,double _cost2go,double _idx_dubin,double _idx_length,double _goal_reach):
      state_pt(_state_pt->copy() ),heuri(_heuri),cost(_cost),cost2go(_cost2go),idx_dubin(_idx_dubin),idx_length(_idx_length),goal_reach(_goal_reach){ };

     void free_point(){ delete state_pt; }
     
     GSnode copy(){
       return GSnode(state_pt->copy(),heuri,cost,cost2go,idx_dubin,idx_length,goal_reach);
     }
  };
};//namespace ends
