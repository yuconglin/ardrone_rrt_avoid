#include "SingleCheck.h"
#include "UavState/GeneralState.h"
#include "obstacle3D.h"
#include <vector>

namespace utils{

bool SingleCheck(const user_types::GeneralState* st_pt, const vector<user_types::obstacle3D>& obstacles)
{
   bool if_collide=false;
   double x_m = st_pt->x;
   double y_m = st_pt->y;
   double z_m = st_pt->z;
   double t_m = st_pt->t;//time
     
   for(int j=0; j!=obstacles.size(); ++j)
   {
      obs3D obs3d=obstacles[j].Estimate(t_m);
      double dis_r=pow(obs3d.x-x_m,2)+pow(obs3d.y-y_m,2)+pow(obs3d.z-z_m,2);
      dis_r = sqrt(dis_r);
     
      if(dis_r < obs3d.r+obstacles[j].delta_r)
      {
	if_collide = true;
	break;
      }
   }
   return if_collide;
}//QuadSingleCheckUncer ends

};//namespace ends
