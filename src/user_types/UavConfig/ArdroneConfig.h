#pragma once
#include "GeneralConfig.h"
namespace user_types{
  
   struct ArdroneConfig:public GeneralConfig{
        double v;
	double vz;
	double yaw_rate;
       
	ArdroneConfig():GeneralConfig(),v(0.),vz(0.),yaw_rate(0.)
        {};
	
	int ParamfromXML(const char* pFilename);
	double MaxAscend();
	double Ascend();
	void NormalizeU(arma::vec::fixed<3>& u);
   };//struct ends

};//Ardrone_rrt_avoid
