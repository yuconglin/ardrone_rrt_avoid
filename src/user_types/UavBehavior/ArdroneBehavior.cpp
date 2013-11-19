#include "ArdroneBehavior.hpp"
#include "UavState/ArdroneState.h"

namespace user_types{

    GeneralState* ArdroneBehavior::InitState(double x_a,double y_a,double z_a,double _t,double _yaw)
    {
        return new ArdroneState(x_a,y_a,z_a,_t,_yaw);
    }
};//namespace ends
