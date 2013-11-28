//#include<cmath>
namespace utils{
//quaternion to RPY,everything goes with my coordinate conversion
void Quat2RPY( double x, double y, double z, double w, double& roll, double& pitch, double& yaw);
//RPY to Quanternions
void RPY2Quat( double roll, double pitch, double yaw, double& x, double& y, double& z, double& w);

};//namespace ends
