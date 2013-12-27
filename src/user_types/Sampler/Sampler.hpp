#pragma once
//forward declaration
namespace user_types{
   struct GeneralState;
}

namespace user_types{
  class Sampler{
	  
   public:
    Sampler();
    virtual void SetParams(GeneralState* root_pt,GeneralState* goal_pt,double _sig_ga)=0;
    virtual void GetSample(double& x_a,double& y_a,double& z_a,GeneralState* root_state_pt,GeneralState* goal_state_pt)=0;
    //set sampling method
    void SetSampleMethod(int _method);
      

   protected:
    double x0,y0,z0,ga0;//the start point
    double sigma_ga;//sigma for gamma angle
    int sample_method;
  };//abstract class Sampler ends

}//namespace user_types ends
