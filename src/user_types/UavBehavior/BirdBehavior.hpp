#include "GeneralBehavior.hpp"

namespace user_types{

   class BirdBehavior:public GeneralBehavior
   {
      GeneralState* InitState(double x_a,double y_a,double z_a,double _t,double _yaw);  
   };

};//namespace ends
