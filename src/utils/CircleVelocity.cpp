#include "CircleVelocity.h"
#include "dubins.h"

#include "UavConfig/GeneralConfig.h"
#include "QuadCfg.h"
#include "armadillo"

namespace utils{
   
    void CircleVelocity(double x_a,double y_a,double z_a,QuadCfg& cfg_start,QuadCfg& cfg_end,int type,user_types::GeneralConfig* config_pt,arma::vec::fixed<3>& de_u)
    {
        double lambda_h = 1.;
        double vec[]={0};
        double rho= config_pt->rho;

	if(type==L_SEG)
	{ 
	   vec[0]= -sin(cfg_start.theta);
	   vec[1]= cos(cfg_start.theta);
	}
	else
	{
	   lambda_h= -1;
	   vec[0]= sin(cfg_start.theta);
	   vec[1]= -cos(cfg_start.theta);
	}
	
	double center[3]= {cfg_start.x+rho*vec[0],cfg_start.y+rho*vec[1],cfg_start.z};
	double c_n= center[0];
	double c_e= center[1];
	double c_d= center[2];
	 
	double de_length= fabs(cfg_end.theta-cfg_start.theta)*rho;
	double h_diff= cfg_end.z-cfg_start.z;
	double tan_gamma= h_diff/de_length;
        //vectors for calculation
	arma::vec::fixed<3> u1, u2, fc1, fc2;
	//calculate starts
	double r_n= x_a; 
	double r_e= y_a;
	double r_d= z_a;
			 
	double pacpr[3]={0,0,0};
	pacpr[0]= 2*(r_n-c_n)/(rho*rho);
	pacpr[1]= 2*(r_e-c_e)/(rho*rho); 
	 
	double pappr[3]={0,0,0};
	double numer= pow(r_n-c_n,2)+pow(r_e-c_e,2);
	pappr[0]= tan_gamma/lambda_h*(r_e-c_e)/numer;
	pappr[1]= -tan_gamma/lambda_h*(r_n-c_n)/numer;
	pappr[2]= 1./rho;
	 
	double ac= pow((r_n-c_n)/rho,2)+pow((r_e-c_e)/rho,2)-1;
	double phi_h= atan2(cfg_start.y-c_e, cfg_end.x-c_n);
	double ap= (r_d-c_d)/rho-tan_gamma/lambda_h*(atan2(r_e-c_e,r_n-c_n)-phi_h ); 
	
	fc1<<pacpr[0] <<pacpr[1] <<pacpr[2];
	fc2<<pappr[0] <<pappr[1] <<pappr[2];
	
	u1<< ac*pacpr[0]+ap*pappr[0]\
	  << ac*pacpr[1]+ap*pappr[1]\
	  << ac*pacpr[2]+ap*pappr[2];
	u2= -lambda_h*cross(fc1,fc2);
	
	de_u= -config_pt->K1*u1 + config_pt->K2*u2;

    }//CircleVelocity ends

};//namespace ends
