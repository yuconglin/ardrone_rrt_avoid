#pragma once
#include <iostream>
#include <fstream>
#include <sstream>
#include "obstacle3D.h"

namespace user_types{
  
struct ObsLog{
  
  //the main logs
  std::vector<obstacle3D> obs_list;
  double r,delta_r;
  int idx;

  //constructor
  ObsLog(double _r,double _del_r):r(_r),delta_r(_del_r),idx(0){};

  //read into obs_list;
  void read_list(const char* filename="/home/yucong/.ros/record1.txt")
  {
     std::ifstream myfile(filename);
     if(!myfile.is_open())
     {
       try{
         throw std::runtime_error ("ObsLog file cannot open");
       }
       catch (std::runtime_error &e) {
         std::cout << "Caught a runtime_error exception: "
                  << e.what () << '\n';
       }
       return;
     }//if !myfile.open()
     
     std::string line;
     double t,vx,vy,vz,w,x,y,z;
     double t0=0.;//initial time
     obs_list.clear();
     
     while(myfile.good() )
     {//read into the list
       getline(myfile,line);
       std::istringstream iss(line, std::istringstream::in); 
       iss>> t;
       if(t>0.)//only start from t>0
       {
	 if(obs_list.empty() ) t0= t;
         //std::cout<< "t= "<<t-t0 <<std::endl; 
	 iss >> vx;
	 iss >> vy;
	 iss >> vz;
	 iss >> w;
	 iss >> x;
	 iss >> y;
	 iss >> z;
	 //conversion
	 double heading= atan2(vy,vx);
	 double speed= sqrt(vx*vx+vy*vy);
//obstacle3D( double _x1, double _x2, double _heading, double _speed, double _x3, double _v_vert, time_t _t, double _r, double _delta_r)
	 obstacle3D obs(x,y,z,vx,vy,vz,t-t0,r,delta_r);
	 obs_list.push_back(obs);
       }
     }//end while
     if(myfile.is_open() ) myfile.close();
  }//read_list ends
  
  //return a obstacle3D in a given time point
  void t_obstacle(double t,obstacle3D& obs)
  {//idx is the index we start to find the time point
     if( obs_list.size()==0 )
     {
       try {
        throw std::runtime_error ("ObsLog:obs_list empty");
       }
       catch (std::runtime_error &e) {
        std::cout << "Caught a runtime_error exception: "
                  << e.what () << '\n';
       }
       return;
     }

     if( obs_list.back().t< t)//obstacle stops moving
     { 
       std::cout<<" back <t "<< std::endl;
       obs= obs_list.back();
       obs.t= t;
       obs.v1= 0., obs.v2= 0., obs.v3= 0.;
     }
     else if( obs_list.front().t> t)//obstacle not start moving yet
     {
       std::cout<<"front >t"<<obs_list.front().t<<std::endl;
       obs= obs_list.front();
       obs.t= t;
       obs.v3=0., obs.v3= 0., obs.v3= 0.; 
     }
     else if( obs_list.back().t>= t && t>= obs_list.front().t)
     {
       std::cout<<"media t"<<std::endl;
       for(int i=idx; i!=obs_list.size(); ++i)
       {
          if( obs_list[i].t<= t && t<= obs_list[i+1].t )
          {
            idx= i;
	    obs= obs_list[i];
	    obs.t= t;
	    break;
          }//if obs_list ends
       }//for ends
     }
     else {;}
  }//t_obstacle ends

  };//ObsLog ends

};//namespace ends
