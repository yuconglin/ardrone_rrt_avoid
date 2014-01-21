#include "armadillo"
#include "ArdroneState.h"
#include <cmath>
//controller gains
#include "controller/config/parrot/config_controller_Parrot.h"
#include "obstacle3D.h"
#include <iostream>

namespace user_types{
   
   int ArdroneState::Update(const arma::vec::fixed<3> u, double dt)
   { //first world reference to local reference
      double ux= u(0)*cos(yaw)+ u(1)*sin(yaw);
      double uy= -u(0)*sin(yaw)+ u(1)*cos(yaw);
      
      double vx_b= vx*cos(yaw)+ vy*sin(yaw);
      double vy_b= -vx*sin(yaw)+ vy*cos(yaw);
      double kp_vx= 5*MULTIROTOR_SPEEDCONTROLLER_VX_KP;
      double kp_vy= 5*MULTIROTOR_SPEEDCONTROLLER_VY_KP;
      double kp_yaw= MULTIROTOR_SPEEDCONTROLLER_YAW_KP;
      double kp_vz= -5*MULTIROTOR_SPEEDCONTROLLER_Z_KP;
      //max
      double vxy_max= 1.0*78.19/69;
      double vz_max= 1/69*10.5;
      double yaw_rate_max= 70./180*M_PI;

      double vx_d,vy_d;
      //vx,x
      if( abs(ux) >vxy_max ) 
	vx_d= (ux < 0 ? -1.0 : 1.0) * vxy_max;
      else 
	vx_d= ux;
      double ax= kp_vx*(vx_d- vx_b)/1.;
      double dx= vx_b*dt+ 0.5*ax*dt*dt;
      vx_b= vx_b+ax*dt;

      //vy,y
      if( abs(uy) >vxy_max ) 
	vy_d= (uy < 0 ? -1.0 : 1.0) * vxy_max;
      else 
	vy_d= uy;
      double ay= kp_vy*(vy_d- vy_b)/1.;
      double dy= vy_b*dt+ 0.5*ay*dt*dt;
      vy_b= vy_b+ay*dt;

      //yaw
      double yaw_d= atan2( u(1),u(0) );
      double dyaw= kp_yaw*(yaw_d-yaw);
      //std::cout<<"yaw= "<< yaw<< std::endl;
      
      dyaw= (dyaw> yaw_rate_max*dt)? yaw_rate_max*dt:dyaw ;
      yaw_rate= dyaw/dt;
      yaw+= dyaw;
      
      //back to world reference frame
      vx= vx_b*cos(yaw)- vy_b*sin(yaw);
      vy= vx_b*sin(yaw) +vy_b*cos(yaw);
      x+= dx*cos(yaw)- dy*sin(yaw);
      y+= dx*sin(yaw)+ dy*cos(yaw);
      //vx= vx_b, vy= vy_b, x+= dx, y+= dy;
      //z
      double vz_d= 0.;
      
      if( abs(u(2)) >vz_max )
	vz_d= (u(2)< 0?-1.0:1.0)* vz_max;
      else
	vz_d= u(2);
      //vz_d*= ratio;
      double az= kp_vz*(vz_d- vz);
      z+= vz*dt+ 0.5*az*dt*dt;
      vz+= az*dt;
      //to print
      //std::cout<<"ax="<<ax<<" ay="<< ay<< std::endl;
      //std::cout<<"vx="<<vx<<" vy= "<<vy<< std::endl;
      //std::cout<<"v="<< sqrt(vx*vx+vy*vy) << std::endl;
      //t
      t+= dt;
      return 0;
   }//Update ends
   
   void ArdroneState::LogData(std::ofstream& file)
   {
      if(!file.is_open() )
      {
	//std::runtime_error("file not open");
	try {
	  throw std::runtime_error ("file not open");
	}
	catch (std::runtime_error &e) {
	  std::cout << "Caught a runtime_error exception: "
		  << e.what () << '\n';
	}
      }
      file<< t<< " "<< x<<" "<< y<<" "<< z<<" "<< yaw<<" "
	  << vx<<" "<< vy<<" "<< vz<<" "<< yaw_rate<< std::endl;
   }//LogData ends
   
   GeneralState* ArdroneState::copy()
   {  //potential memory link
      return new ArdroneState(x,y,z,t,yaw,vx,vy,vz,yaw_rate);
      //return new ArdroneState(x,y,z,t,yaw,vx,vy,vz,yaw_rate,ax,ay,az,a_yaw);
   }

   GeneralState* ArdroneState::InterPolate(double dt)
   {
      double x_a= x+vx*dt;
      double y_a= y+vy*dt;
      double z_a= z+vz*dt;
      double t_a= t+dt;
      double yaw_a= yaw;
      double vx_a= vx;
      double vy_a= vy;
      double vz_a= vz;
      double yaw_rate_a= yaw_rate;
      return new ArdroneState(x_a,y_a,z_a,t_a,yaw_a,vx_a,vy_a,vz_a,yaw_rate_a);
   }//InterPolate ends
   
   void ArdroneState::Print()
   {
      std::cout<<"ArdroneState print: "<<x<<","<<y<<","<<z<<","<<t<<","<<yaw<<","<<vx<<","<<vy<<","<<vz<<","<<yaw_rate<< std::endl;
   }//Print() ends

   obstacle3D ArdroneState::toObs3D(double r,double dr)
   {
     return obstacle3D(x,y,z,vx,vy,vz,t,r,dr);
   }//to Obs3D

  
  
   /*
   int ArdroneState::Update(const arma::vec::fixed<3> u, double dt)
   {
       //first world reference to local reference
       double ux= u(0)*cos(yaw)+ u(1)*sin(yaw);
       double uy= -u(0)*sin(yaw)+ u(1)*cos(yaw);
       
       double vx_b= vx*cos(yaw)+ vy*sin(yaw);
       double vy_b= -vx*sin(yaw)+ vy*cos(yaw);
      
       double kp_vx= 5*MULTIROTOR_SPEEDCONTROLLER_VX_KP;
       double kp_vy= 5*MULTIROTOR_SPEEDCONTROLLER_VY_KP;
       double kp_yaw= MULTIROTOR_SPEEDCONTROLLER_YAW_KP;
       double kp_vz= -5*MULTIROTOR_SPEEDCONTROLLER_Z_KP;
       double vxy_max= 2.0;
       double Tc= 0.3;

       double vx_d,vy_d;
       //std::cout<<"accer:"<< ax<<" "<<ay<<" "<<az<<" "<<a_yaw<< std::endl;

       //vx,x
       if( abs(ux) >vxy_max ) 
	 vx_d= (ux < 0 ? -1.0 : 1.0) * vxy_max;
       else 
	 vx_d= ux;
       
       double de_ax= kp_vx*(vx_d- vx_b)/1.;
       //acceleration change
       ax+= (de_ax-ax)/Tc*dt; 
       vx_b= vx_b+ax*dt;
       double dx= vx_b*dt+ 0.5*ax*dt*dt;
       
       //vy,y
       if( abs(uy) >vxy_max ) 
	 vy_d= (uy < 0 ? -1.0 : 1.0) * vxy_max;
       else 
	 vy_d= uy;
       
       double de_ay= kp_vy*(vy_d- vy_b)/1.;
       //acceleration
       ay+= (de_ay-ay)/Tc*dt;
       vy_b= vy_b+ay*dt;
       double dy= vy_b*dt+ 0.5*ay*dt*dt;

       //yaw
       double yaw_d= atan2( u(1),u(0) );
       double yaw_rate_max= 70./180*M_PI;
       double de_dyaw= kp_yaw*(yaw_d-yaw);
       //acceleration
       a_yaw+= (de_dyaw/dt-a_yaw)/Tc*dt;
       yaw_rate+= a_yaw*dt;
       
       if(abs(yaw_rate) >yaw_rate_max)
	 yaw_rate= (yaw_rate<0?-1.:1.)*yaw_rate_max;

       yaw+= yaw_rate*dt;
       
       //back to world reference frame
       vx= vx_b*cos(yaw)- vy_b*sin(yaw);
       vy= vx_b*sin(yaw) +vy_b*cos(yaw);
       x+= dx*cos(yaw)- dy*sin(yaw);
       y+= dx*sin(yaw)+ dy*cos(yaw);
       //z
       double vz_max= 1.0;
       double vz_d= 0.;
       
       if( abs(u(2)) >vz_max )
	 vz_d= (u(2)< 0?-1.0:1.0)* vz_max;
       else
	 vz_d= u(2);
       
       double de_az= kp_vz*(vz_d- vz);
       az+= (de_az-az)/Tc*dt;
       vz+= az*dt;
       z+= vz*dt+ 0.5*az*dt*dt;
       
       //t
       t+= dt;
       return 0;
   }//Update ends
   */
    
   
};//namespace ends
