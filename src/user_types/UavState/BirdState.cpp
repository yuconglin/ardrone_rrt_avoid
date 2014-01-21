#include "armadillo"
#include "BirdState.h"
#include <cmath>
//controller gains
#include "controller/config/parrot/config_controller_Parrot.h"
#include "obstacle3D.h"
#include <iostream>

namespace user_types{
  
   int BirdState::Update(const arma::vec::fixed<3> u, double dt)
   {  //first world reference to local reference
      double ux= u(0)*cos(yaw)+ u(1)*sin(yaw);
      double uy= -u(0)*sin(yaw)+ u(1)*cos(yaw);
      
      double vx_b= vx*cos(yaw)+ vy*sin(yaw);
      double vy_b= -vx*sin(yaw)+ vy*cos(yaw);
      double kp_vx= 1*MULTIROTOR_SPEEDCONTROLLER_VX_KP;
      double kp_vy= 1*MULTIROTOR_SPEEDCONTROLLER_VY_KP;
      double kp_yaw= MULTIROTOR_SPEEDCONTROLLER_YAW_KP;
      double kp_vz= -1*MULTIROTOR_SPEEDCONTROLLER_Z_KP;
      //max values
      double vxy_max=78.19;
      double yaw_rate_max= 70./180*M_PI;
      double vz_max= 10.5;
      //equations
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
   
   void BirdState::LogData(std::ofstream& file)
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
   
   GeneralState* BirdState::copy()
   {  //potential memory link
      return new BirdState(x,y,z,t,yaw,vx,vy,vz,yaw_rate);
      //return new BirdState(x,y,z,t,yaw,vx,vy,vz,yaw_rate,ax,ay,az,a_yaw);
   }

   GeneralState* BirdState::InterPolate(double dt)
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
      return new BirdState(x_a,y_a,z_a,t_a,yaw_a,vx_a,vy_a,vz_a,yaw_rate_a);
   }//InterPolate ends
   
   void BirdState::Print()
   {
      std::cout<<"BirdState print: "<<x<<","<<y<<","<<z<<","<<t<<","<<yaw<<","<<vx<<","<<vy<<","<<vz<<","<<yaw_rate<< std::endl;
   }//Print() ends

   obstacle3D BirdState::toObs3D(double r,double dr)
   {
     return obstacle3D(x,y,z,vx,vy,vz,t,r,dr);
   }//to Obs3D


};//namespace ends
