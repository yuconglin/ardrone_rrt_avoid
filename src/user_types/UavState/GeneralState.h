#pragma once
#include "armadillo"

namespace user_types{

  struct GeneralState{
     double x;
     double y;
     double z;
     double t;
     double yaw;
     //constructor
     GeneralState():x(0.),y(0.),z(0.),t(0.),yaw(0.){};
     ~GeneralState(){};
     GeneralState(double _x,double _y,double _z,double _t,double _yaw):x(_x),y(_y),z(_z),t(_t),yaw(_yaw){};
     //virtual function 
     virtual int Update(const arma::vec::fixed<3> u, double dt) = 0;
     virtual void LogData(std::ofstream& file ) = 0;
  };//GeneralState ends

};//namespace ends
