#pragma once
#include "armadillo"

//controller
#include "controller/midlevelCnt/Controller_MidLevelCnt.h"
#include "controller/midlevelCnt/Controller_MidLevel_controlModes.h"
#include "Other/jesus_library.h"

namespace Ardrone_rrt_avoid{

   int QuadState::Update(const arma::vec::fixed<3> u, double dt)
   {
      double de_yaw= atan2(u(1),u(0) );
      double yaw_est = jesus_library::mapAnglesToBeNear_PIrads( yaw, d_yaw);
      //for control output
      double pitchco,rollco,dyawco,dzco;
      //controller start
      Controller_MidLevelCnt *control_pt= new Controller_MidLevelCnt;
      double vx_est= vxy*cos(yaw);
      double vy_est= vxy*sin(yaw);
      controlMid.setFeedback( x, y, vx_est, vy_est, yaw_est, z);
      controlMid.setReference( 0.0, 0.0, de_yaw, 0.0, u(0), u(1) );
      controlMid.getOutput( &pitchco, &rollco, &dyawco, &dzco);  
      delete control_pt;
      //controller ends
      //only pitch needs to be reversed with a minus sign
   }//Update ends

};//namespace ends
