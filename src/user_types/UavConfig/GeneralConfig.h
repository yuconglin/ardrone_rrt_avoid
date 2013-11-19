#pragma once
#include "armadillo"
namespace user_types{

  struct GeneralConfig{
     
     double dt;
     double speed;//that is total speed just for planning
     double rho;//turning radius
     //gain matrix for controlling
     arma::mat::fixed<3,3> K1; 
     arma::mat::fixed<3,3> K2;

     GeneralConfig():dt(0.),speed(0.),rho(0.)
     {
       K1= 10*arma::eye<arma::mat>(3,3); 
       K2= 10*arma::eye<arma::mat>(3,3);  	
     }
     ~GeneralConfig(){};
     //get para from xml
     virtual int ParamfromXML(const char* pFilename)= 0;
     virtual double MaxAscend()= 0;
     virtual double Ascend()= 0;
     virtual void NormalizeU(arma::vec::fixed<3>& u)= 0;
  };

};//namespace ends
