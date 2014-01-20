#pragma once
#include "GeneralState.h"
#include "armadillo"
#include "obstacle3D.h"

namespace user_types{

  struct BirdState:public GeneralState{
   double vx;
   double vy;
   double vz;
   double yaw_rate;

   BirdState():GeneralState(),vx(0.),vy(0.),vz(0.),yaw_rate(0.){};
   BirdState(double _x,double _y,double _z,double _t,double _yaw):GeneralState(_x,_y,_z,_t,_yaw),vx(0.),vy(0.),vz(0.),yaw_rate(0.){};
   BirdState(double _x,double _y,double _z,double _t,double _yaw,double _vx,double _vy,double _vz,double _yaw_rate):GeneralState(_x,_y,_z,_t,_yaw),vx(_vx),vy(_vy),vz(_vz),yaw_rate(_yaw_rate){};
   
   //virtual functions
   int Update(const arma::vec::fixed<3> u, double dt); 
   void LogData(std::ofstream& file);
   GeneralState* copy();
   GeneralState* InterPolate(double dt);
   void Print();
   obstacle3D toObs3D(double r,double dr);
     
  /***********************************/
  };//struct ends

};//namespace ends
