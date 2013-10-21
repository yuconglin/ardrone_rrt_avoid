#pragma once
struct QuadCfg{
   double x;
   double y;
   double z;
   double theta;
   
   QuadCfg():x(0.),y(0.),z(0.),theta(0.){

   };

   QuadCfg(double _x,double _y,double _z,double _the):x(_x),y(_y),z(_z),theta(_the){
   };
};//quadCfg ends
