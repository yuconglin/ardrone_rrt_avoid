#pragma once
//generic tree 
#include "tree.hh"
//uav state
#include "UavState/GeneralState.h"
//tree node
#include "UavState/GSnode.h"
//used structs
//not everyone needs to be an object, some can just be pointers
//list for cpp funcions to include
#include "Sampler3D/Sampler3D.hpp"
//#include "SpaceLimit.h"

namespace Ardrone_rrt_avoid{

   class YlClRRT{
     
     private:
       user_types::Sampler3D sampler;
       user_types::SpaceLimit* spaceLimit_pt;
       user_types::GeneralConfig* config_pt;

   }//class YlClRRT ends


}//namespace ends
