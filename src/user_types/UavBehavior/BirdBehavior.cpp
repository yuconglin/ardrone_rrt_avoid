#include "BirdBehavior.hpp"
#include "UavState/BirdState.h"

namespace user_types{

    GeneralState* BirdBehavior::InitState(double x_a,double y_a,double z_a,double _t,double _yaw)
    {
        return new BirdState(x_a,y_a,z_a,_t,_yaw);
    }
};//namespace ends
