#include <iostream>
#include "QuadConfig.hpp"
using namespace std;

int main( int argc, char** argv )
{
  QuadConfig quad_config;
  if(quad_config.s_ReadFromXML()!= -1)
  {
    cout<< "s_vxy_P=" << quad_config.s_vxy_P << endl;
    cout<< "s_vxy_I=" << quad_config.s_vxy_I << endl;
    cout<< "s_vxy_D=" << quad_config.s_vxy_D << endl;
    cout<< "s_vxy_Limit=" << quad_config.s_vxy_Limit << endl;
    cout<< "s_vz_P=" << quad_config.s_vz_P << endl;
    cout<< "s_vz_I=" << quad_config.s_vz_I << endl;
    cout<< "s_vz_D=" << quad_config.s_vz_D << endl;
    cout<< "s_vz_Limit=" << quad_config.s_vz_Limit << endl;
    cout<< "s_yaw_P=" << quad_config.s_yaw_P << endl;
    cout<< "s_yaw_I=" << quad_config.s_yaw_I << endl;
    cout<< "s_yaw_D=" << quad_config.s_yaw_D << endl;
    cout<< "s_yaw_Limit=" << quad_config.s_yaw_Limit << endl;
  }// if quad_config ends
}
