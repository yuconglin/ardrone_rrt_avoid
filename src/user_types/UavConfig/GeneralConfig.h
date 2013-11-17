#pragma once
namespace user_types{

  struct GeneralConfig{
     
     double dt;
     double speed;//that is total speed just for planning
     double rho;//turning radius

     GeneralConfig():dt(0.),speed(0.),rho(0.){};
     ~GeneralConfig(){};
     //get para from xml
     virtual int ParamfromXML(const char* pFilename)= 0;
     virtual double MaxAscend()= 0;
     virtual double Ascend()= 0;
  };

};//namespace ends
