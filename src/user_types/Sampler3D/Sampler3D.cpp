#include "Sampler3D.hpp"
#include "UavConfig/GeneralConfig.h"
#include <boost/random.hpp>
#include <ctime>

namespace user_types{

   Sampler3D::Sampler3D():x0(0.),y0(0.),z0(0.),r0(0.),sigma_r(0.),theta0(0.),sigma_theta(0.),ga0(0.),sigma_ga(0.){ };

   Sampler3D::Sampler3D(double _x0,double _y0,double _z0,double _r0,double _sigr,double _th0, double _sigth, double _ga0, double _sigga):x0(_x0),y0(_y0),z0(_z0),r0(_r0),sigma_r(_sigr),theta0(_th0),sigma_theta(_sigth),ga0(_ga0),sigma_ga(_sigga){ };
  
   void Sampler3D::SetParams(double x_root,double y_root,double z_root,double x_goal,double y_goal,double z_goal)
   { 
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
       gamma0= atan2(Dz,r0);
     }
     else {;}
    
     x0 = x_root;
     y0 = y_root;
     z0 = z_root;
     this->r0 = r0;
    
     sigma_r= 0.5*r0;
     this->theta0 = theta0;
     sigma_theta= 0.125*M_PI;
     ga0 = gamma0;
     //sigma_ga= config_pt->MaxAscend()*0.25;
    
   };//SetParams ends

   void Sampler3D::GetSample(double& x_a,double& y_a,double& z_a,GeneralState* root_state_pt,GeneralState* goal_state_pt)
   {
      boost::mt19937 generator;
      static unsigned int seed = 0;
      generator.seed(static_cast<unsigned int>(std::time(0))+(++seed));
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
