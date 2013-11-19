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
			 QuadCfg cfg_target,//stop quad state
			 const vector<obstacle3D>& obstacles,
                         const user_types::checkParas check_paras,
			 user_types::GeneralConfig* config_pt,
			 std::vector<user_types::GeneralState*> path_log,//path for log
			 double& actual_length//actual length tranversed
			 );

};//namespace ends
