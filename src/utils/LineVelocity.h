#include "armadillo"
#include "QuadCfg.h"
#include "UavConfig/GeneralConfig.h"

namespace utils{

 void LineVelocity(double x,double y,double z,QuadCfg& cfg_start,QuadCfg& cfg_end,user_types::GeneralConfig* config_pt,arma::vec::fixed<3>& de_u);

};//namespace ends
