#pragma once
#include "Sampler.hpp"

namespace user_types{
   struct GeneralState;
}

namespace user_types{

   class SamplerPole:public Sampler{
    
    public:
      void SetParams(GeneralState* root_pt,GeneralState* goal_pt,double _sig_ga);
      void GetSample(double& x_a,double& y_a,double& z_a,GeneralState* root_state_pt,GeneralState* goal_state_pt);

    private:
      double r0;
      double sigma_r;
      double theta0;
      //double sigma_theta;

   };//SamplerPole ends

}
