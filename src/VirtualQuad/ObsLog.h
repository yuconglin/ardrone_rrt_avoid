//a struct to deal with logged quad path, as a moving obstacle to avoid
#pragma once
#include "common_struct.hpp"
#include <fstream>
#include <iostream>

struct ObsLog{
  //the main logs
  vector<obstacle3D> obs_list;
  //read into obs_list;
  void read_list(const char* filename="/home/yucong/.ros/record1.txt")
  {
     std::ifstream myfile(filename);
     if(!myfile.is_open())
     {
       std::cerr<<"obstacle fine unopenable"<<std::endl;
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
	 obstacle3D obs(x,y,heading,speed,z,vz,t-t0,1,1);
	 obs_list.push_back(obs);
       }
     }//end while
     if(myfile.is_open() ) myfile.close();
  }//read_list ends
  
  //return a obstacle3D in a given time point
  void t_obstacle(double t,obstacle3D& obs,int& idx)
  {//idx is the index we start to find the time point
     if( obs_list.size()==0 )
       read_list();
     
     if( obs_list.back().t< t)//obstacle stops moving
     { 
       std::cout<<" back <t "<< std::endl;
       obs= obs_list.back();
       obs.t= t;
       obs.speed= 0;
       obs.v_vert= 0;
     }
     else if( obs_list.front().t> t)//obstacle not start moving yet
     {
       std::cout<<"front >t"<<obs_list.front().t<<std::endl;
       obs= obs_list.front();
       obs.t= t;
       obs.speed= 0;
       obs.v_vert= 0;
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
