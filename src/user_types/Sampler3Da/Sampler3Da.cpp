#include "Sampler3Da.hpp"
#include "SpaceLimit.h"

namespace user_types {
   
   Sampler3Da::Sampler3Da():xl(0),yl(0),x_len(0),y_len(0),the(0),ga0(0),sigma_ga(0){ };

   Sampler3Da::SetParams(double x_root,double y_root,double z_root,double x_goal,double y_goal,double z_goal,user_types::SpaceLimit* spaceLimit_pt,double _len,double _width):x_len(_len),y_len(_len)
   {
     double Dx= fabs(x_goal-x_root);
     double Dy= fabs(y_goal-y_root);
     double Dz= fabs(z_goal-z_root);
     double r0= sqrt(Dx*Dx+Dy*Dy);
     theta= atan2(Dy,Dx);
     xl= -x_l/2;
     yl= 0;
     ga0= atan2(Dz,r0);
     //get the four corners
     //left lower
     double x0= x_root+ y_len/2*sin(theta);
     double y0= y_root- y_len/2*cos(theta);
     double z0= z_root;
     //right lower
     double x1= x_root+ x_len*cos(theta)+ y_len/2*sin(theta);
     double y1= y_root+ x_len*sin(theta)- y_len/2*cos(theta);
     double z1= z_goal;
     //right upper
     double x2= x_root+ x_len*cos(theta)- y_len/2*sin(theta);
     double y2= y_root+ x_len*sin(theta)+ y_len/2*cos(theta);
     double z2= z_goal;
     //left upper
     double x3= x_root- y_len/2*sin(theta);
     double y3= y_root+ y_len/2*cos(theta);
     double z3= z_root;
     //compare with spacelimit
     if( !spaceLimit_pt->TellIn(x0,y0,z0)
       ||!spaceLimit_pt->TellIn(x1,y1,z1)
       ||!spaceLimit_pt->TellIn(x2,y2,z2)
       ||!spaceLimit_pt->TellIn(x3,y3,z3)
       )
     {
       xl= spaceLimit_pt->vertex[0].x;
       yl= spaceLimit_pt->vertex[0].y;
       x_len= fabs(spaceLimit_pt->vertex[1].x-spaceLimit_pt->vertex[0].x);
       y_len= fabs(spaceLimit_pt->vertex[1].y-spaceLimit_pt->vertex[0].y);
       theta= 0;
     }//judge ends
   }//SetParams ends

};//namespace ends
