#pragma once

#include <vector>
//forward declaration
struct quadDubins3D;
struct QuadCfg;
struct obstacle3D;

namespace user_types{
  struct GeneralState;
  struct checkParas;
  struct GeneralConfig;
  struct obstacle3D;
}

namespace utils{
    
  int DubinsTotalCheck(quadDubins3D& db_3d,//the dubins curve
                         user_types::GeneralState* st_init,//initial actual state
			 user_types::GeneralState* st_final,//final state
			 QuadCfg& cfg_target,//stop quad state
			 std::vector<user_types::obstacle3D>& obstacles,
                         user_types::checkParas* checkparas_pt,
			 user_types::GeneralConfig* config_pt,
                         std::vector<user_types::GeneralState*>* path_log_pt=0,//path for log
			 double *actual_length_pt	 
			 );

};//namespace ends
