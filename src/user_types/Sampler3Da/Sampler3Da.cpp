#include "Sampler3Da.hpp"
#include "SpaceLimit.h"
//#include "UavConfig/GeneralConfig.h"
#include <boost/random.hpp>
#include <ctime>
#include <iostream>

namespace user_types {
   
   Sampler3Da::Sampler3Da():x_bot(0),y_bot(0),x_len(0),y_len(0),theta(0){ };

   void Sampler3Da::SetParams(GeneralState* root_state_pt,GeneralState* goal_state_pt,SpaceLimit* spaceLimit_pt,double _width,double _height)
   {
     double x_root= root_state_pt->x;
     double y_root= root_state_pt->y;
     double z_root= root_state_pt->z;
     double x_goal= goal_state_pt->x;
     double y_goal= goal_state_pt->y;
     double z_goal= goal_state_pt->z; 

     double Dx= x_goal-x_root;
     double Dy= y_goal-y_root;
     double Dz= z_goal-z_root;
     double r0= sqrt(Dx*Dx+Dy*Dy);
     x_len= r0;
     y_len= _width;
     z_len= _height;
     theta= atan2(Dy,Dx);
     
     x_bot= x_root;
     y_bot= y_root;
     z_bot= z_root;
     /*
     //get the four corners
     //left lower
     double x0= x_root+ y_len/2*sin(theta);
     double y0= y_root- y_len/2*cos(theta);
     double z0= z_root;
     std::cout<<"x0: "<<x0<<" y0: "<<y0<<" z0: "<<z0<<std::endl;
     //right lower
     double x1= x_root+ x_len*cos(theta)+ y_len/2*sin(theta);
     double y1= y_root+ x_len*sin(theta)- y_len/2*cos(theta);
     double z1= z_goal;
     std::cout<<"x1: "<<x1<<" y1: "<<y1<<" z1: "<<z1<<std::endl;
     //right upper
     double x2= x_root+ x_len*cos(theta)- y_len/2*sin(theta);
     double y2= y_root+ x_len*sin(theta)+ y_len/2*cos(theta);
     double z2= z_goal;
     std::cout<<"x2: "<<x2<<" y2: "<<y2<<" z2: "<<z2<<std::endl;
     //left upper
     double x3= x_root- y_len/2*sin(theta);
     double y3= y_root+ y_len/2*cos(theta);
     double z3= z_root;
     std::cout<<"x3: "<<x3<<" y3: "<<y3<<" z3: "<<z3<<std::endl;
     //compare with spacelimit
     
     if( !spaceLimit_pt->TellIn(x0,y0,z0)
       ||!spaceLimit_pt->TellIn(x1,y1,z1)
       ||!spaceLimit_pt->TellIn(x2,y2,z2)
       ||!spaceLimit_pt->TellIn(x3,y3,z3)
       )
     {
       std::cout<<"out out"<< std::endl;
       x_bot= 0.5*(spaceLimit_pt->vertex[0].x+spaceLimit_pt->vertex[3].x);
       y_bot= 0.5*(spaceLimit_pt->vertex[1].y+spaceLimit_pt->vertex[2].y);
       z_bot= 0.5*(spaceLimit_pt->h_lower+spaceLimit_pt->h_upper);
       x_len= fabs(spaceLimit_pt->vertex[1].x-spaceLimit_pt->vertex[0].x);
       y_len= fabs(spaceLimit_pt->vertex[0].y-spaceLimit_pt->vertex[3].y);
       z_len= fabs(spaceLimit_pt->h_upper-spaceLimit_pt->h_lower);
       std::cout<<"x_bot: "<<x_bot<<" y_bot: "<<y_bot<<" z_bot: "<<z_bot<<std::endl;
       std::cout<<"x_len: "<<x_len<<" y_len: "<<y_len<<" z_len: "<<z_len<<std::endl;
       theta= 0;
     }//judge ends
     */   
   }//SetParams ends

   void Sampler3Da::GetSample(double& x_a,double& y_a,double& z_a,GeneralState* root_state_pt,GeneralState* goal_state_pt)
   {
      boost::mt19937 generator;
      static unsigned int seed = 0;
      generator.seed(static_cast<unsigned int>(std::time(0))+(++seed));
      //sample x,y
      //x
      boost::uniform_real<> x_uniform(0, x_len);
      boost::variate_generator<boost::mt19937&,boost::uniform_real<> > x_uni(generator, x_uniform);
      double xc= x_uni();
      //y
      boost::uniform_real<> y_uniform(-y_len/2, y_len/2);
      boost::variate_generator<boost::mt19937&,boost::uniform_real<> > y_uni(generator, y_uniform);
      double yc= y_uni();
      //transform
      x_a= x_bot+ xc*cos(theta)- yc*sin(theta);
      y_a= y_bot+ xc*sin(theta)+ yc*cos(theta);
      //x_a= x_bot+xc;
      //y_a= y_bot+yc;
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
   
   void Sampler3Da::SetSampleMethod(int _method)
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
