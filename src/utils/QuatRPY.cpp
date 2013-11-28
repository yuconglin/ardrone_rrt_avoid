#include <cmath>
#include "QuatRPY.h"
namespace utils{

void Quat2RPY( double x, double y, double z, double w, double& roll, double& pitch, double& yaw)
{
   yaw= atan2(2*w*z-2*x*y, w*w+x*x-y*y-z*z);
   pitch= asin(-2*x*z-2*w*y);
   roll= atan2(2*w*x-2*y*z, w*w-x*x-y*y+z*z);
}

void RPY2Quat( double roll, double pitch, double yaw, double& x, double& y, double& z, double& w)
{
   roll = roll/2.;
   pitch = pitch/2.;
   yaw = yaw/2.;
   w= -( cos(roll)*cos(pitch)*cos(yaw)+sin(roll)*sin(pitch)*sin(yaw) );
   x= -sin(roll)*cos(pitch)*cos(yaw)+cos(roll)*sin(pitch)*sin(yaw);
   y= cos(roll)*sin(pitch)*cos(yaw)+sin(roll)*cos(pitch)*sin(yaw);
   z= sin(roll)*sin(pitch)*cos(yaw)-cos(roll)*cos(pitch)*sin(yaw);
   double mag= sqrt(w*w+x*x+y*y+z*z);
   w= w/mag;
   x= x/mag;
   y= y/mag;
   z= z/mag;
}

};//namespace ends
