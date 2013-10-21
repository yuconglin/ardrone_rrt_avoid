#pragma once
#include "dubins.h"
//#include "QuadCfg.h"
#include "QuadCfg.h"
#include <cmath>
#include <fstream>
/*
#define L_SEG (0)
#define S_SEG (1)
#define R_SEG (2)

const int DIRDATA[][3] = {
    { L_SEG, S_SEG, L_SEG },
    { L_SEG, S_SEG, R_SEG },
    { R_SEG, S_SEG, L_SEG },
    { R_SEG, S_SEG, R_SEG },
    { R_SEG, L_SEG, R_SEG },
    { L_SEG, R_SEG, L_SEG }
};*/
//extern int DIRDATA[][3];

struct quadDubins3D{
  //the projected path
  DubinsPath path2D;
  //start end and two intermidiate states
  QuadCfg cfg_start;
  QuadCfg cfg_i1;
  QuadCfg cfg_i2;
  QuadCfg cfg_end;
  //length of 3 segments
  double seg_param[3];
  double gamma;
  //how to initialize such curve
  //void Init(const QuadCfg& start, const QuadCfg& end, double rho)
  quadDubins3D(){ };

  quadDubins3D(const QuadCfg& start,const QuadCfg& end,double rho)
  {
    this->cfg_start= start;
    this->cfg_end = end;
    double q0[]= { start.x, start.y, start.theta };
    double q1[]= { end.x, end.y, end.theta };
    dubins_init(q0, q1, rho, &path2D);
    
    double length_2d= dubins_path_length(&path2D);
    //double h_diff= _end.z-_start.z;
    this->gamma= atan2( end.z-start.z, length_2d );
    for(int i=0;i<3;++i)
       seg_param[i]= path2D.param[i]/cos(gamma)*rho;
    //get cfg_i1, cfg_i2
    double p1 = path2D.param[0];
    double p2 = path2D.param[1];
    double qi[3] = { 0, 0, path2D.qi[2] };
    double q2[3]; // end-of segment 2

    const int* types = DIRDATA[path2D.type];
    dubins_segment( p1,  qi,  q1, types[0] );
    dubins_segment( p2,  q1,  q2, types[1] );
    q1[0] = q1[0] * path2D.rho + path2D.qi[0];
    q1[1] = q1[1] * path2D.rho + path2D.qi[1];
    q1[2] = mod2pi(q1[2]);
    q2[0] = q2[0] * path2D.rho + path2D.qi[0];
    q2[1] = q2[1] * path2D.rho + path2D.qi[1];
    q2[2] = mod2pi(q2[2]);
    cfg_i1.theta= q1[2];
    cfg_i2.theta= q2[2];
    cfg_i1.z= start.z+ path2D.param[0]*rho*tan(gamma);
    cfg_i2.z= cfg_i1.z+ path2D.param[1]*rho*tan(gamma);
    cfg_i1.x= q1[0];
    cfg_i1.y= q1[1];
    cfg_i2.x= q2[0];
    cfg_i2.y= q2[1];
  }//Init ends

  //output to a text
  void OutDubins(double stepSize,const char* filename="dubins_record.txt")
  {
     std::ofstream file(filename);
     double x = 0.0;
     double length = dubins_path_length(&path2D);
     //double h_diff= cfg_end.z- cfg_start.z;
     while( x < length ) 
     {
	 double q[3];
	 dubins_path_sample( &path2D, x, q );
	 double h = x*tan(this->gamma)+cfg_start.z;
	 x += stepSize;
	 if(file.is_open())
	   file<< q[0]<<" "<< q[1]<<" "<< h<< std::endl;
     }
     if(file.is_open()) file.close();
  }//OutDubins ends

  double GetHeuriLength() { return seg_param[0]+seg_param[1]+seg_param[2];}

  //find the length closest to a target point
  double CloseLength(double x_p, double y_p, double z_p)
  {
     double x= 0.,p_len= 0.;
     double length= dubins_path_length(&path2D);
     double stepSize= length/100;
     double dis_temp= 1e10,dis;
     while(x<length)
     {
        double q[3];
        dubins_path_sample( &path2D, x, q );
        double h = x*tan(this->gamma)+cfg_start.z;
        dis= sqrt(pow(x_p-q[0],2)+pow(y_p-q[1],2)+pow(z_p-h,2));
        if(dis<dis_temp) 
	{ 
          p_len= x/cos(this->gamma);
	  dis_temp= dis;
	}//if dis ends
	x+= stepSize;
     }//while ends
     dis= sqrt(pow(x_p-cfg_end.x,2)+pow(y_p-cfg_end.y,2)+pow(z_p-cfg_end.z,2) );
     if(dis<dis_temp) 
     { 
        p_len= length/cos(this->gamma);
	dis_temp= dis;
     }//if dis ends

     return p_len;
  }//CloseLength ends

};//struct define ends 
