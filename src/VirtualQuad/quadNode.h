#pragma once
#include "QuadState.h"
struct quadNode{
//node for quad rrt trees
 //basic config
  QuadState state;
  double heuri;//heuristics to the sampled node
  double cost;//cost from root to it
  double cost2go;//cost to the goal
  int idx_dubin;//which dubins curve it belongs to
  int idx_state;//which state in the dubins curve it is
  bool goal_reach;//if accessible from the goal
  //constructor
  quadNode(){
     heuri=0.;
     cost=0.;
     cost2go=0.;
     idx_dubin=-1;
     idx_state=-1;
     goal_reach= false;
  };
  //constructor
  quadNode(double _x,double _y,double _z,double _the,double _v,double _vz,double _t
         ) 
  {
     //QuadState new_st(_x,_y,_z,_the,_v,_vz,_t);
     this->state = QuadState(_x,_y,_z,_the,_v,_vz,_t);
     heuri=0.;
     cost=0.;
     cost2go=0.;
     idx_dubin=-1;
     idx_state=-1;
     goal_reach= false;

  };

  quadNode( const QuadState& _state )
  {
     this->state= _state;
     heuri=0.;
     cost=0.;
     cost2go=0.;
     idx_dubin=-1;
     idx_state=-1;
     goal_reach= false;

  };
};//quadNode ends
