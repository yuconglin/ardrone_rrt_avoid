#include "CollectCheck.h"
#include "UavState/GeneralState.h"
#include "ObsCollect.h"
#include <vector>

namespace utils{

   bool CollectCheck(const user_types::GeneralState* st_pt, user_types::ObsCollect& obscollect)
   {
      //bool if_collide= false;
      
      double x_m = st_pt->x;
      double y_m = st_pt->y;
      double z_m = st_pt->z;
      double t_m = st_pt->t;//time
      //obstacle3D 	
      for(int j=0; j!=obscollect.obs_3ds.size(); ++j)
      {
	 user_types::obs3D obs3d= obscollect.obs_3ds[j].Estimate(t_m);
	 double dis_r=pow(obs3d.x-x_m,2)+pow(obs3d.y-y_m,2)+pow(obs3d.z-z_m,2);
	 dis_r = sqrt(dis_r);
	
	 if(dis_r < obs3d.r+ obscollect.obs_3ds[j].delta_r)
	 {
	   //if_collide = true;
	   //break;
	   return true;
	 }
      }
      //obstacle2D
      for(int j=0; j!=obscollect.obs_2ds.size(); ++j)
      {
         user_types::obstacle2D obs2d= obscollect.obs_2ds[j];
         double dis_r=pow(obs2d.x-x_m,2)+pow(obs2d.y-y_m,2);
	 dis_r= sqrt(dis_r);
	 if(dis_r< obs2d.r+ obs2d.delta_r)
	 {
           return true;
	 }
      }
      //polygon static obstacles
      
      return false;
   }//CollectCheck ends
};
