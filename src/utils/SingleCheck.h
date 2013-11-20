#pragma once
//#include "UavState/GeneralState.h"
//#include "obstacle3D.h"
#include <vector>
//forward declaration
namespace user_types{
   struct GeneralState;
   struct obstacle3D;
}

namespace utils{

  bool SingleCheck(const user_types::GeneralState* st_pt, std::vector<user_types::obstacle3D>& obstacles);
  //true if collide
};//namespaces ends
