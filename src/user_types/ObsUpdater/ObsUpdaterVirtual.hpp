#pragma once
#include "ObsUpdater.hpp"
#include <vector>
#include "ObsLog.h"

namespace user_types{
  class ObsUpdaterVirtual:public ObsUpdater{
    public:
      ObsUpdaterVirtual(std::vector<std::string> filenames,double _r,double _del_r);
      ~ObsUpdaterVirtual();
      inline bool SeeObsUpdate(){return true;};
      inline void SetObsUpdateFalse(){/*do nothing*/};
      void UpdateObs(std::vector<obstacle3D>& obs3ds,double t); 
    private:
      std::vector<user_types::ObsLog*> obslogs;
  };//class ends

};//namespace ends
