#pragma once

#include <vector>
//forward declaration
struct quadDubins3D;
struct QuadCfg;

namespace user_types{
  struct GeneralState;
  struct checkParas;
  struct GeneralConfig;
  struct ObsCollect;
  struct SpaceLimit;
}

namespace utils{
    
  int DubinsTotalCheck(quadDubins3D& db_3d,//the dubins curve
                         user_types::GeneralState* st_init,//initial actual state
			 user_types::GeneralState* st_final,//final state
			 QuadCfg& cfg_target,//stop quad state
			 user_types::ObsCollect& obs_collect,
                         user_types::checkParas* checkparas_pt,
			 user_types::GeneralConfig* config_pt,
			 user_types::SpaceLimit* spaceLimit_pt,
                         std::vector<user_types::GeneralState*>* path_log_pt=0,//path for log
			 double *actual_length_pt=0	 
			 );

};//namespace ends
