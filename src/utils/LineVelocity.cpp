#include "LineVelocity.h"
#include "UavConfig/GeneralConfig.h"
#include "armadillo"
#include "QuadCfg.h"

namespace utils{
  
  void LineVelocity(double x,double y,double z,QuadCfg& cfg_start,QuadCfg& cfg_end,user_types::GeneralConfig* config_pt,arma::vec::fixed<3>& de_u)
  {
    double x_start= cfg_start.x;
    double y_start= cfg_start.y;
    double z_start= cfg_start.z;
    double x_end= cfg_end.x;
    double y_end= cfg_end.y;
    double z_end= cfg_end.z;
    
    double dis= sqrt(pow(x_start-x_end,2)+pow(y_start-y_end,2)+pow(z_start-z_end,2));
    
    double phi= atan2(y_end-y_start,x_end-x_start);
    double gam= asin( (z_end-z_start)/dis );

    arma::vec::fixed<3> n_lon,n_lat,u;

    n_lon<<-sin(phi)<<cos(phi)<<0.;
    n_lat<<cos(phi)*sin(gam)<<sin(phi)*sin(gam)<<-cos(gam);
    
    double a_lon= n_lon(0)*(x-x_start)+n_lon(1)*(y-y_start)+n_lon(2)*(z-z_start);
    double a_lat= n_lat(0)*(x-x_start)+n_lat(1)*(y-y_start)+n_lat(2)*(z-z_start);
    //unnormailised velocity
    de_u = -config_pt->K1*(a_lon*n_lon+a_lat*n_lat)+ config_pt->K2*cross(n_lat,n_lon); 
  };//LineVelocity ends

};
