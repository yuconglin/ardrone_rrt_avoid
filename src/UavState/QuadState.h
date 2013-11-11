#pragma once
#include "GeneralState.h"
#include "armadillo"

//declaration of classes used
class Controller_MidLevelCnt;

namespace Ardrone_rrt_avoid{

  struct QuadState:public GeneralState{
    double yaw;
    double vx;
    double vy;
    double vz;
    double yaw_rate;
    //constructor
    QuadState():GeneralState(),yaw(0.),vx(0.),vy(0.),vz(0.),yaw_rate(0.){};
    QuadState(double _x,double _y,double _z,double _t,double _yaw,double _vx,double _vy,double _vz,double _yaw_rate):GeneralState(_x,_y,_z,_t),yaw(_yaw),vx(_vx),vy(_vy),vz(_vz),yaw_rate(_yaw_rate){};
    //virtual function: state update
    int Update(const arma::vec::fixed<3> u, double dt);
    void LogData(std::ofstream& file);
  };//QuadState ends
};//namespace ends
