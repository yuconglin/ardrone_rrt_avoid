#pragma once
#include "GeneralState.h"
#include "armadillo"

namespace Ardrone_rrt_avoid{

  struct QuadState:public GeneralState{
    double theta;
    double vxy;
    double vz;
    //constructor
    QuadState(double _x,double _y,double _z,double _t,double _theta,double _vxy,double _vz):
      GeneralState(_x,_y,_z,_t),theta(_theta),vxy(_vxy),vz(_vz){};
    //virtual function: state update
    int Update(const arma::vec::fixed<3> u, double dt)
    {

    }//Update ends

  };//QuadState ends
};//namespace ends
