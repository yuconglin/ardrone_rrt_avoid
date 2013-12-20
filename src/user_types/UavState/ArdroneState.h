#pragma once
#include "GeneralState.h"
#include "armadillo"
#include "obstacle3D.h"

//declaration of classes used
class Controller_MidLevelCnt;

namespace user_types{

  struct ArdroneState:public GeneralState{
    //double yaw;
    double vx;
    double vy;
    double vz;
    double yaw_rate;
    //accelerations
    //double ax;
    //double ay;
    //double az;
    //double a_yaw;
    //constructor
    ArdroneState():GeneralState(),vx(0.),vy(0.),vz(0.),yaw_rate(0.){
       //ax= 0.;
       //ay= 0.;
       //az= 0.;
       //a_yaw= 0.;
    };
    
    ArdroneState(double _x,double _y,double _z,double _t,double _yaw):GeneralState(_x,_y,_z,_t,_yaw),vx(0.),vy(0.),vz(0.),yaw_rate(0.){
       //ax= 0.;
       //ay= 0.;
       //az= 0.;
       //a_yaw= 0.;
    };
    
    ArdroneState(double _x,double _y,double _z,double _t,double _yaw,double _vx,double _vy,double _vz,double _yaw_rate):GeneralState(_x,_y,_z,_t,_yaw),vx(_vx),vy(_vy),vz(_vz),yaw_rate(_yaw_rate){
       //ax= 0.;
       //ay= 0.;
       //az= 0.;
       //a_yaw= 0.;
    };

   // ArdroneState(double _x,double _y,double _z,double _t,double _yaw,double _vx,double _vy,double _vz,double _yaw_rate,double _ax,double _ay,double _az,double _a_yaw):GeneralState(_x,_y,_z,_t,_yaw),vx(_vx),vy(_vy),vz(_vz),yaw_rate(_yaw_rate){};
    
    //virtual function: state update
    int Update(const arma::vec::fixed<3> u, double dt); 
    void LogData(std::ofstream& file);
    GeneralState* copy();
    GeneralState* InterPolate(double dt);
    void Print();
    obstacle3D toObs3D();
  };//ArdroneState ends

};//namespace ends
