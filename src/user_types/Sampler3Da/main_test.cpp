#include "Sampler3Da.hpp"
#include <fstream>
#include "SpaceLimit.h"
#include "UavState/ArdroneState.h"

int main(int argc, char** argv)
{
  user_types::Sampler3Da sampler;
  //spacelimit
  std::vector<user_types::point2D> vec_rect;
  double h=19*0.6096,r=4,l=-4;
  vec_rect.push_back( user_types::point2D(0,l) );
  vec_rect.push_back( user_types::point2D(h,l) );
  vec_rect.push_back( user_types::point2D(h,r) );
  vec_rect.push_back( user_types::point2D(0,r) );
  double h_max= 2., h_min= 0.5;
  user_types::SpaceLimit* spaceLimit_pt= new user_types::SpaceLimit(h_max,h_min,vec_rect); 
  //set params
  user_types::GeneralState* root_pt= new user_types::ArdroneState(1,0,1,0,M_PI/18.);
  user_types::GeneralState* goal_pt= new user_types::ArdroneState(11,1,1,0,0);
  sampler.SetParams(root_pt,goal_pt,spaceLimit_pt,3,2);
  //sample multiple
  std::ofstream myfile("sample_test.txt");
  double x_a,y_a,z_a;
  sampler.SetSampleMethod(1);
  for(int i=0;i!=10000;++i)
  {
     sampler.GetSample(x_a,y_a,z_a,root_pt,goal_pt);
     myfile<<x_a<<" "<<y_a<<" "<<z_a<< std::endl;
  }
  myfile.close();
  delete spaceLimit_pt;
  delete root_pt;
  delete goal_pt;
}//main ends
