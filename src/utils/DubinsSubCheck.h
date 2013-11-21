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

  int DubinsSubCheck(quadDubins3D& db_3d,//the dubins curve
                     user_types::GeneralState* st_init,//initial actual state
		     user_types::GeneralState* st_final,//final state
		     QuadCfg &cfg_target,//stop quad state
		     std::vector<user_types::obstacle3D>& obstacles,
		     user_types::checkParas* checkparas_pt,
		     user_types::GeneralConfig* config_pt,
		     std::vector<user_types::GeneralState*>* path_log_pt,//path for log
		     double* actual_length_pt,//actual length tranversed
		     int idx_seg//which segment: 0,1,2
      );

};//namespace ends
