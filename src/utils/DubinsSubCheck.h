#pragma once
//quad releated
//#include "quadDubins3D.h"
//#include "QuadCfg.h"
//utils
//user defined types
//#include "UavState/GeneralState.h"
//#include "obstacle3D.h"
//#include "checkParas.h"
//#include "UavConfig/GeneralConfig.h"
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
		     const std::vector<user_types::obstacle3D>& obstacles,
		     const user_types::checkParas& check_paras,
		     user_types::GeneralConfig* config_pt,
		     std::vector<user_types::GeneralState*> path_log,//path for log
		     double& actual_length,//actual length tranversed
		     int idx_seg//which segment: 0,1,2
      );

};//namespace ends
