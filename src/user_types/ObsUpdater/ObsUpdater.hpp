#pragma once
#include "obstacle3D.h"
#include <vector>

namespace user_types{
  class ObsUpdater{
    public:
      virtual bool SeeObsUpdate()= 0;
      virtual void SetObsUpdateFalse()= 0;
      virtual void UpdateObs(std::vector<obstacle3D>& obs3ds); 
    private:
      std::vector<bool> if_updates;

  }//class ObsUpdater ends

};//namespace ends
