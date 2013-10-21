#include "assert.h"
#include "ticpp.h"
#include "VirtualQuad.hpp"
#include "armadillo"
#include <cmath>
#include <iostream>
//#include "yucong_rrt_avoid/Dubin_msg.h"
//#include "quadDubins3D.h"

int VirtualQuad::ParamFromXML(const char* pFilename)
{
   try
   {
     ticpp::Document doc(pFilename);
      // actually load the information
     doc.LoadFile();
      //the element to start
     ticpp::Element *child= doc.FirstChildElement("quadrotor_param")->FirstChildElement();
     //an iterator
     ticpp::Iterator<ticpp::Element> iter(child);
     for( iter=iter; iter!=iter.end(); ++iter )
     {
         std::string strName;
         iter->GetValue(&strName);
	 //
	 if( strName== "velocity_xy" )
	   config.v= atof(iter->GetText().c_str() );
         if( strName== "velocity_z" )
	   config.vz= atof(iter->GetText().c_str() );
         if( strName== "dt")
	   config.dt= atof(iter->GetText().c_str() );
         if( strName== "yaw_rate")
 	   config.yaw_rate= atof(iter->GetText().c_str() );
     }//for child ends

   }
   catch(ticpp::Exception& error)
   {
     cerr << "Error: " << error.m_details << endl;
     return 2;                 // signal error
   }

   return 0;
}//ParamFromXML ends

void VirtualQuad::SetConfig(double _v,double _w,double _vz,double _dt)
{
   config.v= _v;
   config.yaw_rate= _w;
   config.vz= _vz;
   config.dt= _dt;
   //must successful reading
   if(ParamFromXML("/home/yucong/.ros/param.xml")!=0)
     std::runtime_error("ParamFromXML error");
   if(config.s_ReadFromXML()!=0)
     std::runtime_error("config s_ReadFromXML error");
   
   this->rho= config.v/config.yaw_rate;
   std::cout<<"rho= "<<rho << std::endl;
   this->speed= sqrt(config.v*config.v+config.vz*config.vz);
   end_r= max(speed*1*config.dt,0.15);
   std::cout<<"end_r= "<<end_r<<std::endl;
   ds_check= speed*config.dt;
   ds_insert= 5*ds_check;
}//SetConfig ends

void VirtualQuad::StateUpdate(const QuadState& st,const arma::colvec& u, QuadState& st_next,const double dt)
{  
   //assert(dt<1.0);
   double v_d,vz_d,accer,w_d;
   double p=2.8/3;
   //vz and z
   if( abs(u(2)) >config.s_vz_Limit ) 
     vz_d= (u(2) < 0 ? -1.0 : 1.0) * config.s_vz_Limit;
   else vz_d= u(2);
   
   accer= p*config.s_vz_P*(vz_d- st.vz)/1.;
   //accer= accer+ config.s_vz_D*(accer-st.az);
   //st_next.az= accer;
   st_next.vz = st.vz+accer*dt;
   st_next.z= st.z+st.vz*dt+0.5*accer*dt*dt;
   //yaw
   double d_the= atan2(u(1),u(0) ); 
   if(abs(d_the-st.theta)>config.s_yaw_Limit*dt) 
   {
     st_next.theta= ( d_the-st.theta<0? -1.:1.)*config.s_yaw_Limit*dt+ st.theta; 
     w_d= ( d_the-st.theta<0? -1.:1.)*config.s_yaw_Limit;
   }
   else{
     st_next.theta= d_the;
     w_d= (d_the-st.theta)/dt;
   }
   //v 
   v_d= sqrt( u(0)*u(0)+u(1)*u(1) );
   if( v_d> config.s_vxy_Limit )
     v_d= config.s_vxy_Limit;
   accer= p*config.s_vxy_P*(v_d- st.v)/1.;
   //std::cout<<"v accer= "<< accer << std::endl;
   //st_next.av= accer;
   st_next.v= st.v+ accer*dt;

   if(w_d!=0)
   {
     st_next.x= st.x+ (st.v+0.5*accer*dt)/w_d*( sin(w_d*dt+st.theta)-sin(st.theta) );
     st_next.y= st.y- (st.v+0.5*accer*dt)/w_d*( cos(w_d*dt+st.theta)-cos(st.theta) );
   }
   else
   {
     st_next.x= st.x+ (st.v+0.5*accer*dt)*cos(st.theta)*dt;
     st_next.y= st.y+ (st.v+0.5*accer*dt)*sin(st.theta)*dt;
   }
   st_next.t= st.t+dt;
}//StateUpdate ends

bool VirtualQuad::IfCanAvoid(const QuadState& st_current, const obstacle3D& obs)
{//to estimate if the quad can avoid a given obstacle
   bool if_left= false, if_right= false;
   double xx,yy,zz,uv1,uv2,uv3,vx,x_x,vv,happ_dt;
   double t1= 0.5*M_PI/config.s_yaw_Limit; //time to turn 90 degree
   cout<<"t1= "<<t1<<endl;
   //x1
   double x1= st_current.x+0.5*(st_current.v+config.v)*(sin(st_current.theta+config.s_yaw_Limit*1)-sin(st_current.theta) );
   x1= x1+config.v*(sin(st_current.theta+0.5*M_PI)-sin(st_current.theta+config.s_yaw_Limit*1));
   //y1
   double y1= st_current.y-0.5*(st_current.v+config.v)*(cos(st_current.theta+config.s_yaw_Limit*1)-cos(st_current.theta) );
   y1= y1+config.v*(cos(st_current.theta+0.5*M_PI)-cos(st_current.theta+config.s_yaw_Limit*1));
   double z1= st_current.z+ 0.5*(config.vz+st_current.vz)*1+config.vz*(t1-1);
   //obstacle predict
   obs3D obs3d=obs.Estimate(t1+st_current.t);
   double dis_r=pow(obs3d.x-x1,2)+pow(obs3d.y-y1,2)+pow(obs3d.z-z1,2);
   dis_r= sqrt(dis_r);
   cout<<"dis_r: "<<dis_r<<endl;
   if(dis_r< obs.r+obs.delta_r)
   //already collided at curve part
     if_left =false;
   else
   {
   //otherwise, let's check the straight line parts
     xx= x1-obs3d.x;
     yy= y1-obs3d.y;
     zz= z1-obs3d.z;
     uv1= config.v*cos(st_current.theta+0.5*M_PI)-obs.speed*cos(obs.heading);
     uv2= config.v*sin(st_current.theta+0.5*M_PI)-obs.speed*sin(obs.heading);
     uv3= config.vz-obs.v_vert;
     vx = xx*uv1+yy*uv2+zz*uv3;
     x_x = xx*xx+yy*yy+zz*zz;
     vv = uv1*uv1+uv2*uv2+uv3*uv3;
     happ_dt = -vx/vv;
     if(happ_dt<= 0 )
       if_left= true;
     else{
       double min_dis = sqrt( x_x+vv*happ_dt*happ_dt+2*vx*happ_dt );
       if(min_dis< obs.r+obs.delta_r)
	 if_left= false;
       else 
	 if_left= true;
     }//else ends
   }
   if(if_left) return true;
   //otherwise check right
    x1= st_current.x+0.5*(st_current.v+config.v)*(sin(st_current.theta-config.s_yaw_Limit*1)-sin(st_current.theta) );
    x1= x1+config.v*(sin(st_current.theta-0.5*M_PI)-sin(st_current.theta-config.s_yaw_Limit*1));
    //y1
    y1= st_current.y-0.5*(st_current.v+config.v)*(cos(st_current.theta-config.s_yaw_Limit*1)-cos(st_current.theta) );
    y1= y1+config.v*(cos(st_current.theta-0.5*M_PI)-cos(st_current.theta-config.s_yaw_Limit*1));
  
   //obstacle predict
   dis_r=pow(obs3d.x-x1,2)+pow(obs3d.y-y1,2)+pow(obs3d.z-z1,2);
   dis_r= sqrt(dis_r);
   if(dis_r< obs.r+obs.delta_r)
     //already collided at curve part
       if_right =false;
   else
   {
     //otherwise, let's check the straight line parts
       uv1= config.v*cos(st_current.theta-0.5*M_PI)-obs.speed*cos(obs.heading);
       uv2= config.v*sin(st_current.theta-0.5*M_PI)-obs.speed*sin(obs.heading);
       vx = xx*uv1+yy*uv2+zz*uv3;
       x_x = xx*xx+yy*yy+zz*zz;
       vv = uv1*uv1+uv2*uv2+uv3*uv3;
       happ_dt = -vx/vv;
       if(happ_dt<= 0 )
	 if_right= true;
       else{
	 double min_dis = sqrt( x_x+vv*happ_dt*happ_dt+2*vx*happ_dt );
	 if(min_dis< obs.r+obs.delta_r)
	   if_right= false;
	 else 
	   if_right= true;
       }//else ends
   }
   return if_right; 
}//IfCanAvoid ends
