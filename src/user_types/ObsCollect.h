#pragma once
#include "point2D.h"
#include "obstacle2D.h"
#include "obstacle3D.h"
#include <vector>
namespace user_types{
  
  struct ObsCollect{
     
     std::vector<obstacle2D> obs_2ds;
     std::vector<obstacle3D> obs_3ds;
     //std::vector< std::vector<point2D> > polygons;
  };

};//namespace ends
