#pragma once

#include "PointInPoly.h"
#include "point2D.h"  

namespace user_types{
 
  struct SpaceLimit{
     //height upper limit
     double h_upper;
     double h_lower;
     //vertex for the geofencing polygon
     std::vector<point2D> vertex;
     //vertex for polygon
     //constructor
     SpaceLimit():h_upper(0.),h_lower(0.){};
     ~SpaceLimit(){};
     SpaceLimit(double _h_upper,double _h_lower,const std::vector<point2D> _vertex)
     :h_upper(_h_upper),h_lower(_h_lower),vertex(_vertex){};//SpaceLimit ends
     //to judge if a point is within the SpaceLimit
     bool TellIn(double x, double y,double z)
     { //in return true;
       if( z<= h_lower ||z>= h_upper) 
	 return false;
       if( !utils::PointInPoly(vertex,x,y) )
	 return false;
       return true;
     }//TellIn ends
  
  };//SpaceLimit ends

}//namespace ends
