#include "UavConfig/GeneralConfig.h"
#include "QuadCfg.h"
#include "armadillo"

namespace utils{
  
   void CircleVelocity(double x_a,double y_a,double z_a,QuadCfg& cfg_start,QuadCfg& cfg_end,int type,user_types::GeneralConfig* config_pt,arma::vec::fixed<3>& de_u);

};//namespace ends
