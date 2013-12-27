#pragma once
#include "Sampler.hpp"

namespace user_types{
   struct GeneralState;
}

namespace user_types{
   class SamplerRect:public Sampler{
    
    public:
      void SetParams(GeneralState* root_pt,GeneralState* goal_pt,double _sig_ga);
      void GetSample(double& x_a,double& y_a,double& z_a,GeneralState* root_state_pt,GeneralState* goal_state_pt);
    
    private:
      double x_len;
      double y_len;
      double theta;
   };//class Sampler
}
