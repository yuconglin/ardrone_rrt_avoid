#include "SamplerPole.hpp"
#include "UavState/GeneralState.h"
#include <boost/random.hpp>
#include <ctime>

namespace user_types{
   void SamplerPole::SetParams(GeneralState* root_state_pt,GeneralState* goal_state_pt,double _sig_ga)
   {
      double x_root= root_state_pt->x;
      double y_root= root_state_pt->y;
      double z_root= root_state_pt->z;
      double x_goal= goal_state_pt->x;
      double y_goal= goal_state_pt->y;
      double z_goal= goal_state_pt->z;
      //
      double Dx= x_goal-x_root;
      double Dy= y_goal-y_root;
      double Dz= z_goal-z_root;
      double theta0= atan2(Dy,Dx);
     
      double r0,gamma0;
      if(sample_method ==0)
      {
        r0= sqrt(Dx*Dx+Dy*Dy+Dz*Dz);
        gamma0= asin(Dz/r0);
      }
      else if(sample_method ==1)
      {
        r0= sqrt(Dx*Dx+Dy*Dy);
        //std::cout<<"r0= "<<r0<< std::endl;
        gamma0= atan2(Dz,r0);
      }
      else {;}
      //std::cout<<"Dx= "<< Dx <<" Dy= "<< Dy<<" r0= "<< r0<<std::endl;
      x0 = x_root;
      y0 = y_root;
      z0 = z_root;
      this->r0 = r0;
    
      sigma_r= 0.5*r0;
      
      this->theta0 = theta0;
      //sigma_theta= 0.125*M_PI;
      ga0 = gamma0;
      sigma_ga= _sig_ga;
   }

   void SamplerPole::GetSample(double& x_a,double& y_a,double& z_a,GeneralState* root_state_pt,GeneralState* goal_state_pt)
   {
      boost::mt19937 generator;
      static unsigned int seed = 0;
      generator.seed(static_cast<unsigned int>(std::time(0))+(++seed));
      
      //std::cout<<"sigma_r= "<< sigma_r <<std::endl;  
      boost::normal_distribution<> r_distribution(r0,sigma_r);
      boost::variate_generator<boost::mt19937&,boost::normal_distribution<> > r_nor(generator, r_distribution);  
      double r= r_nor();
      
      boost::uniform_real<> the_uniform(theta0-M_PI/2.0, theta0+M_PI/2.0);
      boost::variate_generator<boost::mt19937&,boost::uniform_real<> > the_nor(generator, the_uniform);
      double theta= the_nor();
      if(sample_method == 0)
      {
	boost::normal_distribution<> ga_distribution(ga0, sigma_ga);
	boost::variate_generator<boost::mt19937&,boost::normal_distribution<> > ga_nor(generator, ga_distribution); 
	double ga= ga_nor();

	x_a= x0+ r*cos(theta)*cos(ga);
	y_a= y0+ r*sin(theta)*cos(ga);
	z_a= z0+ r*sin(ga);
      }
      else if(sample_method==1)
      {
	x_a= x0+ r*cos(theta);
	y_a= y0+ r*sin(theta);
	double d_x= x0- x_a;
	double d_y= y0- y_a;
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

	z_a= z0+ 1./n_z*( n_x*d_x+n_y*d_y );
      }

   }//GetSample ends
}//namespace ends
