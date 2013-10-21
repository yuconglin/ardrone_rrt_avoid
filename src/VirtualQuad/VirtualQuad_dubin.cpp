#include "VirtualQuad.hpp"
#include "quadDubins3D.h"
#include "QuadCfg.h"
#include "common_struct.hpp"
#include "funcs.hpp"
#include "assert.h"
#include "yucong_rrt_avoid/DubinSeg_msg.h"
//#define L_SEG (0)
//#define S_SEG (1)
//#define R_SEG (2)

//extern int DIRDATA[][3]; 
bool QuadSingleCheckTUncer(const QuadState& st, const vector<obstacle3D>& obstacles);

int VirtualQuad::DubinsSubCheck(quadDubins3D& db_3d,const QuadState& st_init,const QuadCfg& cfg_target,int idx_seg,QuadState& st_next,const vector<obstacle3D>& obstacles)
{//check along straight line or circle curve
    assert(idx_seg==0||idx_seg==1||idx_seg==2);
    const int* types = DIRDATA[db_3d.path2D.type];
    int type= types[idx_seg];
    assert( type == L_SEG || type == S_SEG || type == R_SEG );

    bool if_colli= false;
    double length= 0;
    double lambda_h = 1.;
    int result=0;

    QuadCfg cfg1,cfg2;
    QuadState st_temp;

    if(idx_seg==0){
	cfg1= db_3d.cfg_start;
	cfg2= db_3d.cfg_i1;
    }
    else if(idx_seg==1){
        cfg1= db_3d.cfg_i1;
	cfg2= db_3d.cfg_i2;
    }
    else{
	cfg1= db_3d.cfg_i2;
	cfg2= db_3d.cfg_end;
    }
    //cout<<"cfg1: "<<cfg1.x<<" "<<cfg1.y<<" "<<cfg1.z<<" "<< endl;
    //cout<<"cfg2: "<<cfg2.x<<" "<<cfg2.y<<" "<<cfg2.z<<" "<< endl;
    //to get the distance along the dubins curve from the start for st_init,cfg_target and cfg2
    double s_init= db_3d.CloseLength(st_init.x,st_init.y,st_init.z);
    double s_target= db_3d.CloseLength(cfg_target.x,cfg_target.y,cfg_target.z);
    double s_end= db_3d.CloseLength(cfg2.x,cfg2.y,cfg2.z);
    //cout<<"s_init: "<<s_init<<" s_target: "<<s_target<<" s_end: "<<s_end<<endl;
    //to get the dis along dubin between them
    
    int n_seg= floor(length/ds_check);
    //vector<state3D> path_log;
    //double init_end_dis=sqrt(pow(cfg2.x-st_now.x,2)+pow(cfg2.y-st_now.y,2)+pow(cfg2.z-st_now.z,2));
    st_now= st_init;
    
    if( type==L_SEG || type==R_SEG )
    {    /*
	 if( type==L_SEG )
           cout<<"left circle"<<endl;
	 else
           cout<<"right circle"<<endl;
         */
         double vec[]={0,0,0};
	 if(type==L_SEG)
	 { 
	  // cout<<"L_SEG"<<endl;
	   vec[0]= -sin(cfg1.theta);
	   vec[1]= cos(cfg1.theta);
	 }
	 else
	 {
	   //cout<<"R_SEG"<<endl;
	   lambda_h= -1;
	   vec[0]= sin(cfg1.theta);
	   vec[1]= -cos(cfg1.theta);
	 }
	 double center[3]= {cfg1.x+rho*vec[0],cfg1.y+rho*vec[1],cfg1.z};
	 double c_n= center[0];
	 double c_e= center[1];
	 double c_d= center[2];
         //vectors for calculation
	 arma::colvec u, u1, u2, fc1, fc2,v_target,v_end;
         //arma::colvec u_pre;

	 double speed= sqrt(config.v*config.v+config.vz*config.vz);
         //while loop
	 while(1)
	 {
            double r_n= st_now.x; 
	    double r_e= st_now.y;
	    double r_d= st_now.z;
	   		    
	    double pacpr[3]={0,0,0};
	    pacpr[0]= 2*(r_n-c_n)/(rho*rho);
	    pacpr[1]= 2*(r_e-c_e)/(rho*rho);
	    
	    double pappr[3]={0,0,0};
	    double numer= pow(r_n-c_n,2)+pow(r_e-c_e,2);
	    pappr[0]= tan(db_3d.gamma)/lambda_h*(r_e-c_e)/numer;
	    pappr[1]= -tan(db_3d.gamma)/lambda_h*(r_n-c_n)/numer;
	    pappr[2]= 1./rho;

	    double ac= pow((r_n-c_n)/rho,2)+pow((r_e-c_e)/rho,2)-1;
	    double phi_h= atan2(cfg1.y-c_e, cfg1.x-c_n);
	    double ap= (r_d-c_d)/rho-tan(db_3d.gamma)/lambda_h*(atan2(r_e-c_e,r_n-c_n)-phi_h ); 
	    fc1<<pacpr[0] <<pacpr[1] <<pacpr[2];
	    fc2<<pappr[0] <<pappr[1] <<pappr[2];
	    u1<< ac*pacpr[0]+ap*pappr[0]\
	      << ac*pacpr[1]+ap*pappr[1]\
	      << ac*pacpr[2]+ap*pappr[2];
	    u2= -lambda_h*cross(fc1,fc2);
	    u= -K1*u1+K2*u2;
            double u_mag= sqrt(u(0)*u(0)+u(1)*u(1)+u(2)*u(2));
            if( u_mag!=0. )
	    { //normailize
	      double cons= speed/u_mag;
              u<< u(0)*cons<< u(1)*cons << u(2)*cons;
              StateUpdate(st_now,u,st_temp,config.dt);
	      //StateUpdatePD(st_now,u,u-u_pre,st_temp,config.dt);
	      //u_pre= u;
	      st_now= st_temp;
              length+= speed*config.dt;
	      //collision checking
	      if( floor(length/ds_check)> n_seg )
	      {
		 //cout<<"length: "<<length<<endl;
		 if(QuadSingleCheckTUncer(st_now,obstacles))
		 {
		   dubin_actual_length+= length;
		   if_colli= true;
		   break;
		 }
		 else
		   path_log.push_back(st_now);
	      }
	      else
		 path_log.push_back(st_now);

	      n_seg= floor(length/ds_check);

	    }//if u_mag ends
	    //std::cout<<"st_now: "<<st_now.x<<" "<<st_now.y<<" "<<st_now.z<<std::endl;
            //terminal judge
	    //target reached
            double target_dis=sqrt(pow(cfg_target.x-st_now.x,2)+pow(cfg_target.y-st_now.y,2)+pow(cfg_target.z-st_now.z,2));
	    v_target<<cfg_target.x-st_now.x<<cfg_target.y-st_now.y<<cfg_target.z-st_now.z;
	    //test dot(u,v_target)
            //if( dot(u,v_target)<=0 ) 
	    //  cout<< target_dis/end_r <<" "<< length-(s_target-s_init) <<endl;

	    if(  dot(u,v_target)<=0&&target_dis<= 3*end_r ||target_dis<end_r 
	       ||dot(u,v_target)<=0&&length> s_target-s_init		    
	      ) 
	    {
	      //cout<< target_dis/end_r <<" "<< length-(s_target-s_init) <<endl;
	      //cout<<"target here"<<endl;
	      result= 1;
	      break;
	    }
            //curve end reached
	    //
	    double end_dis=sqrt(pow(cfg2.x-st_now.x,2)+pow(cfg2.y-st_now.y,2)+pow(cfg2.z-st_now.z,2));
	    v_end<<cfg2.x-st_now.x<<cfg2.y-st_now.y<<cfg2.z-st_now.z;  
	    
	    if( dot(u,v_end)<=0&&end_dis<= 3*end_r || end_dis<=end_r
	       ||dot(u,v_end)<=0 && length> s_end-s_init
	       ||length> 3*(s_end-s_init) 
	      ) 
	      break;
            //in this case, result = 0
	}//while ends
    }//if R_SEG or L_SEG ends
   
    if( type==S_SEG )
    { //cout<<"straight line:"<< endl;
      double x_start=cfg1.x;
      double y_start=cfg1.y;
      double z_start=cfg1.z;
      double x_end=cfg2.x;
      double y_end=cfg2.y;
      double z_end=cfg2.z;
      double dis= sqrt(pow(x_start-x_end,2)+pow(y_start-y_end,2)+pow(z_start-z_end,2));
		  
      double phi= atan2(y_end-y_start,x_end-x_start);
      double gam= asin( (z_end-z_start)/dis );
  
      arma::colvec n_lon,n_lat,u,v_target,v_end;
      //arma::colvec u_pre;

      n_lon<<-sin(phi)<<cos(phi)<<0.;
      n_lat<<cos(phi)*sin(gam)<<sin(phi)*sin(gam)<<-cos(gam);
      //while starts
      while(1)
      {
        double x_now=st_now.x;
	double y_now=st_now.y;
	double z_now=st_now.z;
	//double the= st_now.theta;
	//double ga= st_now.gamma;
        double a_lon= n_lon(0)*(x_now-x_start)+n_lon(1)*(y_now-y_start)+n_lon(2)*(z_now-z_start);
        double a_lat= n_lat(0)*(x_now-x_start)+n_lat(1)*(y_now-y_start)+n_lat(2)*(z_now-z_start);
        //unnormailised velocity
        u = -K1*(a_lon*n_lon+a_lat*n_lat)+K2*cross(n_lat,n_lon);
        double u_mag= sqrt(u(0)*u(0)+u(1)*u(1)+u(2)*u(2));
	//std::cout<<"u: "<<u(0)/u_mag<<" "<<u(1)/u_mag<<" "<<u(2)/u_mag<<std::endl;
        if( u_mag!=0. )
	{
          double cons= speed/u_mag;
	  u<< u(0)*cons<< u(1)*cons << u(2)*cons;
          StateUpdate(st_now,u,st_temp,config.dt);
	  //StateUpdatePD(st_now,u,u-u_pre,st_temp,config.dt);
	  //u_pre= u;
	  st_now= st_temp;
          length+= speed*config.dt;
          //collision checking
	  if( floor(length/ds_check)> n_seg )
	  {
	     //cout<<"length: "<<length<<endl;
	     if(QuadSingleCheckTUncer(st_now,obstacles))
	     {
	       dubin_actual_length+= length;
	       if_colli= true;
	       break;
	     }
	     else
	       path_log.push_back(st_now);
	  }
	  else
	     path_log.push_back(st_now);

	  n_seg= floor(length/ds_check);

	}//if u_mag ends
        //std::cout<<"st_now: "<<st_now.x<<" "<<st_now.y<<" "<<st_now.z<<" "\
		 <<st_now.vz <<std::endl;
	//terminal judge
	//target reached
	double target_dis=sqrt(pow(cfg_target.x-st_now.x,2)+pow(cfg_target.y-st_now.y,2)+pow(cfg_target.z-st_now.z,2));
	v_target<<cfg_target.x-st_now.x<<cfg_target.y-st_now.y<<cfg_target.z-st_now.z;
        //test dot(u,v_target)
        //if( dot(u,v_target)<=0 ) 
	//cout<< target_dis/end_r <<" "<< length-(s_target-s_init) <<endl;

        if( dot(u,v_target)<=0&&target_dis<= 3*end_r ||target_dis<end_r 
	 ||dot(u,v_target)<=0&&length> s_target-s_init		    
	) 
        {
	  //cout<<"target here"<<endl;
          //cout<< target_dis/end_r <<" "<< length-(s_target-s_init) <<endl;
	  result= 1;
	  break;
        }
        //curve end reached
        //
        double end_dis=sqrt(pow(cfg2.x-st_now.x,2)+pow(cfg2.y-st_now.y,2)+pow(cfg2.z-st_now.z,2));
        v_end<<cfg2.x-st_now.x<<cfg2.y-st_now.y<<cfg2.z-st_now.z;  
      
        if( dot(u,v_end)<=0&&end_dis<= 3*end_r || end_dis<=end_r
	  ||dot(u,v_end)<=0 && length> s_end-s_init 
	  ||length> 3*(s_end-s_init) 
	  ) 
	  break;

      }//while ends

    }//if S_SEG ends
    st_next= st_now;
    dubin_actual_length+= length;
    //cout<<"length: "<<length<<endl;
    //return if_colli;
    if(if_colli) result= -1;
    return result;
}//DubinsSubCheck ends

int VirtualQuad::DubinsTotalCheck(double t_sample,quadDubins3D& db_3d,const QuadState& st_init,const QuadCfg& cfg_target,QuadState& st_next,QuadState& st_sample,const vector<obstacle3D>& obstacles,int idx_seg)
{//return 1:free from collision,0:collided after half length,-1:collided before half length
    //if st_init is at the target,return true
    dubin_actual_length= 0.;
    if(!path_log.empty() ) path_log.clear();
    
    if( sqrt(pow(st_init.x-cfg_target.x,2)+pow(st_init.y-cfg_target.y,2)+pow(st_init.z-cfg_target.z,2))<end_r )
    {
      std::cout <<"already there"<<std::endl;
      st_next= st_init;
      return 1;
    }

    QuadCfg cfgs[]={db_3d.cfg_start,db_3d.cfg_i1,db_3d.cfg_i2,db_3d.cfg_end};
    double d[4];
    //d[3]= sqrt(pow(cfgs[3].x-st_init.x,2)+pow(cfgs[3].y-st_init.y,2)+pow(cfgs[3].z-st_init.z,2) );
    if(idx_seg== -1)//that means we need to evaluate which segment it belongs to ;
    {
      int idx= -1;
      double dis_temp = 1e8;
      for(int i=0;i<4;++i)
      {
	  double dis= pow(cfgs[i].x-st_init.x,2)+pow(cfgs[i].y-st_init.y,2)+pow(cfgs[i].z-st_init.z,2);
	  d[i]= sqrt(dis);
	  
	  if( db_3d.seg_param[i]==0) 
	  {
	    //cout<<"i= "<<i<<endl;
	    continue;
	  }

	  if(d[i]<dis_temp)
	  {
	    dis_temp= d[i];
	    idx= i;
	  }//if dis ends
      }//for int i ends
     	
      if(idx==1||idx==2)
      {
	  if( d[idx-1]/(db_3d.seg_param[idx-1]+1e-10)<d[idx+1]/(db_3d.seg_param[idx]+1e-10) )
	    idx_seg= idx-1;
	  else
	    idx_seg= idx;
      }
      else
	  idx_seg= idx;//if idx ends
    }//if idx_seg ends
    //std::cout<<"plan total idx_seg: "<<idx_seg<<std::endl;
    assert(idx_seg==0||idx_seg==1||idx_seg==2||idx_seg==3);
    int colli= 1;
    int result;
    QuadState st_first =st_init;
      
    for(int i= idx_seg;i<3;++i)
    {
       result = DubinsSubCheck(db_3d,st_first,cfg_target,i,st_next,obstacles);
       //if(if_colli) break;
       if(result==-1)
       {//collision
         if( dubin_actual_length>= 0.5*(db_3d.seg_param[0]+db_3d.seg_param[1]+db_3d.seg_param[2]) )
	    colli= 0;
	 else
	    colli= -1;
	 break;
       }
       else if(result== 1)
       {//target reached without collision
         //st_first= st_next;
         break;
       }
       else if(result== 0)//target not reached,no collision
       {
         st_first= st_next;
       }
       else {;}//nothing
    }//for int i ends
    //get st_sample, the state at t_sample
    if( t_sample/config.dt> path_log.size() )
    {
      QuadState qd_st;
      st_sample= qd_st;
    }
    else
      st_sample= path_log[floor(t_sample/config.dt)];
    //return collision check result
    //std::cout<<"colli: "<<colli<<" colli total log size: "<<path_log.size()<<std::endl;
    return colli; 
}//DubinsTotalCheck ends

void VirtualQuad::DubinToMsg(const quadDubins3D& db_3d,const QuadCfg& cfg_target,yucong_rrt_avoid::DubinSeg_msg& db_msg)
{
   //const int* types = DIRDATA[db_3d.path2D.type];
   db_msg.d_dubin.type= db_3d.path2D.type;
   //start
   db_msg.d_dubin.start.x= db_3d.cfg_start.x;
   db_msg.d_dubin.start.y= db_3d.cfg_start.y;
   db_msg.d_dubin.start.z= db_3d.cfg_start.z;
   db_msg.d_dubin.start.theta= db_3d.cfg_start.theta;
   //end
   db_msg.d_dubin.end.x= db_3d.cfg_end.x;
   db_msg.d_dubin.end.y= db_3d.cfg_end.y;
   db_msg.d_dubin.end.z= db_3d.cfg_end.z;
   db_msg.d_dubin.end.theta= db_3d.cfg_end.theta;
   //cfg_i1
   db_msg.d_dubin.pt_i1.x= db_3d.cfg_i1.x;
   db_msg.d_dubin.pt_i1.y= db_3d.cfg_i1.y;
   db_msg.d_dubin.pt_i1.z= db_3d.cfg_i1.z;
   db_msg.d_dubin.pt_i1.theta= db_3d.cfg_i1.theta;
   //cfg_i2
   db_msg.d_dubin.pt_i2.x= db_3d.cfg_i2.x;
   db_msg.d_dubin.pt_i2.y= db_3d.cfg_i2.y;
   db_msg.d_dubin.pt_i2.z= db_3d.cfg_i2.z;
   db_msg.d_dubin.pt_i2.theta= db_3d.cfg_i2.theta;
   //target
   db_msg.stop_pt.x= cfg_target.x;
   db_msg.stop_pt.y= cfg_target.y;
   db_msg.stop_pt.z= cfg_target.z;
}//DubinToMsg ends


bool QuadSingleCheckTUncer(const QuadState& st, const vector<obstacle3D>& obstacles)
{
   bool if_collide=false;
   double x_m = st.x;
   double y_m = st.y;
   double z_m = st.z;
   double t_m = st.t;//time
   //cout<<"mb "<<x_m<<" "<<y_m<<" "<<z_m<<" "<< t_m <<endl;
   
   for(int j=0; j!=obstacles.size(); ++j)
   {
      obs3D obs3d=obstacles[j].Estimate(t_m);
      double dis_r=pow(obs3d.x-x_m,2)+pow(obs3d.y-y_m,2)+pow(obs3d.z-z_m,2);
      dis_r = sqrt(dis_r);
      //cout<<j<<" "<< dis_r <<endl;

      if(dis_r < obs3d.r+obstacles[j].delta_r)
      {
	//cout<<"collide "<<x_m<<" "<<obs3d.x<<" "<<y_m<<" "<<obs3d.y<<" "<<z_m<<" "<<obs3d.z<<endl;
	//cout<<"mb collide "<< dis_r <<endl;
	if_collide = true;
	break;
      }
   }
   return if_collide;
}//QuadSingleCheckUncer ends

