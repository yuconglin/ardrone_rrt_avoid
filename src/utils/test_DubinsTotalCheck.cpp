//test codes for DubinsTotalCheck function
#include "DubinsTotalCheck.h"
//quad required
#include "quadDubins3D.h"
#include "QuadCfg.h"
//user defined types
#include "UavState/GeneralState.h"
#include "UavState/ArdroneState.h"
#include "ObsCollect.h"
#include "checkParas.h"
#include "UavConfig/GeneralConfig.h"
//utils
#include "GetRho.h"

int main(int argc, char** argv)
{
   double x_s = 0;
   double y_s = 0;
   double z_s = 0.5;
   double the_s= 0.;
   QuadCfg start(x_s,y_s,z_s,the_s);

   double x_e= x_s+ 10;
   double y_e= y_s+ 5;
   double z_e= z_s+ 4;
   double the_e= 0.;
   QuadCfg end(x_e,y_e,z_e,the_e);
   //get rho
   double rho= 0;
   utils::GetRho(rho,"/home/yucong/.ros/param.xml");
   std::cout<<"rho= "<<rho<<std::endl;
   
   quadDubins3D db_3d(start,end,rho);
   db_3d.OutDubins(0.1);

   user_types::GeneralState* st_init= new user_types::ArdroneState(x_s,y_s,z_s,0,the_s);
   user_types::GeneralState* st_final= new user_types::ArdroneState(x_e,y_e,z_e,0,the_e);
   user_types::ObsCollect obs_collect;
   user_types::GeneralConfig* config_pt= new user_types::ArdroneConfig();
   config_pt->ParamfromXML("/home/yucong/.ros/param.xml");
   user_types::checkParas(double _end_r,double _ds_check,double _ds_insert)

}//main ends

