#pragma once
#include "UavState/GeneralState.h"

namespace user_types{

   class GeneralBehavior
   {
     public:
      ~GeneralBehavior(){};
      virtual GeneralState* InitState(double x_a,double y_a,double z_a,double _t,double _yaw)=0;   
   };//class ends
}//namespaces ends
