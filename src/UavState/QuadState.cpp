#include "armadillo"
#include "QuadState.h"
//controller
//#include "controller/midlevelCnt/Controller_MidLevelCnt.h"
//#include "controller/midlevelCnt/Controller_MidLevel_controlModes.h"
//#include "Other/jesus_library.h"

namespace Ardrone_rrt_avoid{

   int QuadState::Update(const arma::vec::fixed<3> u, double dt)
   {
      double ratio= 1.;
      double kp_vxy= 0.3*7.1521;
      double vxy_max= 2.0;
      
      double vx_d,vy_d;
      //vx,x
      if( abs(u(0)) >vxy_max ) 
        vx_d= (u(0) < 0 ? -1.0 : 1.0) * vxy_max;
      else 
	vx_d= u(0);
      vx_d*= ratio;
      double ax= kp_vxy*(vx_d- vx)/1.;
      vx= vx+ax*dt;
      x= x+ vx*dt+ 0.5*ax*dt*dt;
      
      //vy,y
      if( abs(u(1)) >vxy_max ) 
        vy_d= (u(1) < 0 ? -1.0 : 1.0) * vxy_max;
      else 
	vy_d= u(1);
      vy_d*= ratio;
      double ay= kp_vxy*(vy_d- vy)/1.;
      vy= vy+ay*dt;
      y= y+ vy*dt+ 0.5*ay*dt*dt;

      //yaw
      double yaw_d= atan2( u(1),u(0) );
      double kp_yaw= 0.1*5.6;
      double yaw_rate_max= 70./180*M_PI;
      double dyaw= kp_yaw*(yaw_d-yaw);
      //std::cout<<"yaw= "<< yaw<< std::endl;
      
      dyaw= (dyaw> yaw_rate_max*dt)? yaw_rate_max*dt:dyaw ;
      yaw_rate= dyaw/dt;
      yaw+= dyaw;

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
