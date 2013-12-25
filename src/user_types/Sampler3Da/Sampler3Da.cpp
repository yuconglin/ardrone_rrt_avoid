#include "Sampler3Da.hpp"
#include "SpaceLimit.h"
//#include "UavConfig/GeneralConfig.h"
#include <boost/random.hpp>
#include <ctime>

namespace user_types {
   
   Sampler3Da::Sampler3Da():xl(0),yl(0),x_len(0),y_len(0),the(0),ga0(0),sigma_ga(0){ };

   Sampler3Da::SetParams(double x_root,double y_root,double z_root,double x_goal,double y_goal,double z_goal,user_types::SpaceLimit* spaceLimit_pt,double _len,double _width):x_len(_len),y_len(_len)
   {
     double Dx= fabs(x_goal-x_root);
     double Dy= fabs(y_goal-y_root);
     double Dz= fabs(z_goal-z_root);
     double r0= sqrt(Dx*Dx+Dy*Dy);
     theta= atan2(Dy,Dx);
     
     x_bot= x_root;
     y_bot= y_root;
     z_bot= z_root;
 
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
       x_bot= 0.5*(spaceLimit_pt->vertex[0].x+spaceLimit_pt->vertex[1].x);
       y_bot= 0.5*(spaceLimit_pt->vertex[0].y+spaceLimit_pt->vertex[1].x);
       z_bot= 0.5*(spaceLimit_pt->h_lower+spaceLimit_pt->h_upper);
       x_len= fabs(spaceLimit_pt->vertex[1].x-spaceLimit_pt->vertex[0].x);
       y_len= fabs(spaceLimit_pt->vertex[1].y-spaceLimit_pt->vertex[0].y);
       z_len= fabs(spaceLimit_pt->h_upper-spaceLimit_pt->h_lower);
       theta= 0;
     }//judge ends
   
   }//SetParams ends

   Sampler3Da::GetSample(double& x_a,double& y_a,double& z_a,GeneralState* root_state_pt,GeneralState* goal_state_pt)
   {
      boost::mt19937 generator;
      static unsigned int seed = 0;
      generator.seed(static_cast<unsigned int>(std::time(0))+(++seed));
      //sample x,y
      //x
      boost::uniform_real<> x_uniform(-x_len/2, x_len/2);
      boost::variate_generator<boost::mt19937&,boost::uniform_real<> > x_uni(generator, x_uniform);
      double xc= x_uni();
      //y
      boost::uniform_real<> y_uniform(0, y_len);
      boost::variate_generator<boost::mt19937&,boost::uniform_real<> > y_uni(generator, y_uniform);
      double yc= y_uni();
      //transform
      x_a= x_bot+ xc*cos(theta)+ yc*sin(theta);
      y_a= y_bot- xc*sin(theta)+ yc*cos(theta);
      
      if(sample_method== 0)
      {
        //y
	boost::uniform_real<> z_uniform(z_bot-z_len/2, z_bot+z_len/2);
	boost::variate_generator<boost::mt19937&,boost::uniform_real<> > z_uni(generator, z_uniform);
	z_a= z_uni();	  
      }
      else if(sample_method==1)
      {
	double d_x= x_bot- x_a;
	double d_y= y_bot- y_a;
	//get normal vector
	double x_r= root_state_pt->x;
	double y_r= root_state_pt->y;
	double z_r= root_state_pt->z;
	double x_g= goal_state_pt->x;
	double y_g= goal_state_pt->y;
	double z_g= goal_state_pt->z;
	      
	double n_x= -1.*(x_g-x_r)*(z_g-z_r);
	double n_y= -1.*(y_g-y_r)*(z_g-z_r);
	double n_z= pow(x_g-x_r,2)+pow(y_g-y_r,2);
	assert(n_z > 0 );

	z_a= z_bot+ 1./n_z*( n_x*d_x+n_y*d_y );
      }

   }//GetSample ends
   
   void Sampler3D::SetSampleMethod(int _method)
   {
     if( _method!=0&& _method!=1)
     {
       try {
        throw std::runtime_error ("method not 0 or 1!");
       }//try end
       catch (std::runtime_error &e) {
        std::cout << "Caught a runtime_error exception: "
                  << e.what () << '\n';
	//default to 1
	sample_method= 1;
       }//catch ends   
     }
     sample_method = _method;
   }//SetSampleMethod ends
		
};//namespace ends
