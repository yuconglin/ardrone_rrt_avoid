#include "armadillo"
#include "QuadState.h"
//controller
//#include "controller/midlevelCnt/Controller_MidLevelCnt.h"
//#include "controller/midlevelCnt/Controller_MidLevel_controlModes.h"
//#include "Other/jesus_library.h"

namespace Ardrone_rrt_avoid{

   int QuadState::Update(const arma::vec::fixed<3> u, double dt)
   { //first world reference to local reference
      double ux= u(0)*cos(yaw)+ u(1)*sin(yaw);
      double uy= -u(0)*sin(yaw)+ u(1)*cos(yaw);
      
      double vx_b= vx*cos(yaw)+ vy*sin(yaw);
      double vy_b= -vx*sin(yaw)+ vy*cos(yaw);
      
      double ratio= 1.;
      double kp_vxy= 0.3*7.1521;
      double vxy_max= 2.0;
      
      double vx_d,vy_d;
      //vx,x
      if( abs(ux) >vxy_max ) 
        vx_d= (ux < 0 ? -1.0 : 1.0) * vxy_max;
      else 
	vx_d= ux;
      vx_d*= ratio;
      double ax= kp_vxy*(vx_d- vx_b)/1.;
      vx_b= vx_b+ax*dt;
      //x= x+ vx*dt+ 0.5*ax*dt*dt;
      double dx= vx_b*dt+ 0.5*ax*dt*dt;
      
      //vy,y
      if( abs(uy) >vxy_max ) 
        vy_d= (uy < 0 ? -1.0 : 1.0) * vxy_max;
      else 
	vy_d= uy;
      vy_d*= ratio;
      double ay= kp_vxy*(vy_d- vy_b)/1.;
      vy_b= vy_b+ay*dt;
      //y= y+ vy*dt+ 0.5*ay*dt*dt;
      double dy= vy_b*dt+ 0.5*ay*dt*dt;

      //yaw
      double yaw_d= atan2( u(1),u(0) );
      double kp_yaw= 0.1*5.6;
      double yaw_rate_max= 70./180*M_PI;
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
      //z
      double kp_vz= 0.5*1.3339;
      double vz_max= 1.0;
      double vz_d= 0.;
      
      if( abs(u(2)) >vz_max )
	vz_d= (u(2)< 0?-1.0:1.0)* vz_max;
      else
	vz_d= u(2);
      vz_d*= ratio;
      double az= kp_vz*(vz_d- vz);
      vz+= az*dt;
      z+= vz*dt+ 0.5*az*dt*dt;
      
      //t
      t+= dt;
      return 0;
   }//Update ends

   void QuadState::LogData(std::ofstream& file)
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

};//namespace ends
