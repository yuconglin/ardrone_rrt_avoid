#pragma once
struct QuadState{
  double x;
  double y;
  double z;
  double theta;
  double v;//velocity on x,y plane
  //double av;//acceleration in xy
  double vz; // vertical velocity
  //double az; //acceleration in z
  double t;//time
  QuadState():x(0.),y(0.),z(0.),theta(0.){ 
     v= 0.;
     vz= 0.;
     t= 0.;
     //av= 0.;
     //az= 0.;
  };
  QuadState(double _x,double _y,double _z,double _the,double _v,double _vz,double _t):x(_x),y(_y),z(_z),theta(_the),v(_v),vz(_vz),t(_t){
     //av= 0.;
     //az= 0.;
 };//
};
