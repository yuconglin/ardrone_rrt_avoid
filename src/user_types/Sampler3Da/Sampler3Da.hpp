#include "UavState/GeneralState.h"

//forward declaration
namespace user_types{
   struct SpaceLimit;
}

namespace user_types {

  class Sampler3Da{
     public:
      Sampler3Da();
      inline void SetSigmaGa(double _sigma_ga){this->sigma_ga=_sigma_ga;}
      //set params from root and goal
      void SetParams(double x_root,double y_root,double z_root,double x_goal,double y_goal,double z_goal,user_types::SpaceLimit* spaceLimit_pt);
      //GetSample
      void GetSample(double& x_a,double& y_a,double& z_a,GeneralState* root_state_pt,GeneralState* goal_state_pt);
      //set sampling method
      void SetSampleMethod(int _method);

     private:
      double x_bot;
      double y_bot;
      double z_bot;
      double x_len;
      double y_len;
      double z_len;
      double theta;
      double ga0;
      double sigma_ga;
      //sample method
      int sample_method;

  };//Sampler3Da ends

};//namespace ends
