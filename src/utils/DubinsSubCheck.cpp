#include "DubinsSubCheck.h"
//quad releated
#include "quadDubins3D.h"
#include "QuadCfg.h"
//utils
#include "CollectCheck.h"
#include "LineVelocity.h"
#include "CircleVelocity.h"
//user defined types
#include "UavState/GeneralState.h"
#include "ObsCollect.h"
#include "checkParas.h"
#include "SpaceLimit.h"
#include "UavConfig/GeneralConfig.h"
//other libraries
#include "armadillo"
//std lib
#include <vector>

using namespace user_types;
using namespace std;

namespace utils{

    int DubinsSubCheck(quadDubins3D& db_3d,//the dubins curve
                     user_types::GeneralState* st_init,//initial actual state
		     user_types::GeneralState* st_final,//final state
		     QuadCfg& cfg_target,//stop quad state
		     user_types::ObsCollect& obs_collect,
		     user_types::checkParas* checkparas_pt,
		     user_types::GeneralConfig* config_pt,
                     user_types::SpaceLimit* spaceLimit_pt,
		     std::vector<user_types::GeneralState*>* path_sub_pt,//path for log
		     double* sub_length_pt,//actual length tranversed
		     int idx_seg//which segment: 0,1,2
     )//don't forget to delete if needed
     {
        if(!(idx_seg==0||idx_seg==1||idx_seg==2) ){
	  try {
	    throw std::runtime_error ("idx_seg should be 0,1 or 2");
	  }
	  catch (std::runtime_error &e) {
	    std::cout << "Caught a runtime_error exception: "
		      << e.what () << '\n';
	  }
        }//if ends
        const int* types = DIRDATA[db_3d.path2D.type];
        int type= types[idx_seg];
        if(!(type == L_SEG || type == S_SEG || type == R_SEG)){
          try{
            throw std::runtime_error("type should be L_SEG,S_SEG or R_SEG.");
	  }
	  catch(std::runtime_error &e){
            std::cout<< "Caught a runtime_error execption: "<< e.what()<< '\n';
	  }
	}//if ends
        //set to default
	if(path_sub_pt) path_sub_pt->clear();
	*sub_length_pt= 0.;
        
	bool if_colli= false;
        //double lambda_h = 1.;
        int result=0;

        QuadCfg cfg1,cfg2;
    
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
        double s_init= db_3d.CloseLength(st_init->x,st_init->y,st_init->z);
        double s_target= db_3d.CloseLength(cfg_target.x,cfg_target.y,cfg_target.z);
        double s_end= db_3d.CloseLength(cfg2.x,cfg2.y,cfg2.z);

	GeneralState* st_now= st_init->copy();
        int n_seg= floor(*sub_length_pt/checkparas_pt->ds_check);
        arma::vec::fixed<3> u, v_target, v_end;
	
	//the while loop
	while(1)
	{  //get u
           if(type== L_SEG||type== R_SEG)
           {
	     CircleVelocity(st_now->x,st_now->y,st_now->z,cfg1,cfg2,type,config_pt,u );
	   }//if type==L_SEG or R_SEG ends
	   else //type== S_SEG
	   { 
	     LineVelocity(st_now->x,st_now->y,st_now->z,cfg1,cfg2,config_pt,u);
	   }//else ends
           //normalize u
	   config_pt->NormalizeU(u);
	   //cout<<"u: "<<u(0)<<" "<<u(1)<<" "<<u(2)<< endl;
	   //state update
	   GeneralState* st_pre= st_now->copy();
           //std::cout<<"st_now: "<<st_now->x <<" "<< st_now->y <<" "<< st_now->z <<std::endl; 
	   //std::cout<<"u: "<< u(0)<<" "<<u(1)<<" "<<u(2)<< std::endl;
	   st_now->Update(u,config_pt->dt);
           *sub_length_pt+= sqrt(pow(st_now->x - st_pre->x,2)+pow(st_now->y - st_pre->y,2)+pow(st_now->z - st_pre->z,2) ); 
	   delete st_pre;
           
	   //collision check
           if( floor(*sub_length_pt/checkparas_pt->ds_check)> n_seg )
	   {
	     //if( SingleCheck(st_now,obstacles) )
	     if( CollectCheck(st_now,obs_collect)
	       ||!spaceLimit_pt->TellIn(st_now->x,st_now->y,st_now->z)
	       )
	     {
	        //dubin_actual_length+= length;
	        if_colli= true;
	        break;
	     }
	   }
	   
	   if(path_sub_pt) 
	   {  
	     GeneralState* st_temp= st_now->copy();
	     path_sub_pt->push_back(st_temp);
	   }
	   
	   n_seg= floor(*sub_length_pt/checkparas_pt->ds_check);

	   //end conditions
           double target_dis=sqrt(pow(cfg_target.x-st_now->x,2)+pow(cfg_target.y-st_now->y,2)+pow(cfg_target.z-st_now->z,2));
	   v_target<< cfg_target.x-st_now->x << cfg_target.y-st_now->y << cfg_target.z-st_now->z;
	   
	   if(  dot(u,v_target)<=0&&target_dis<= 3*checkparas_pt->end_r ||target_dis< checkparas_pt->end_r 
	      ||dot(u,v_target)<=0&& *sub_length_pt > s_target-s_init		    
	     ) 
	   {
	     result= 1;
	     break;
	   }
           
	   double end_dis=sqrt(pow(cfg2.x-st_now->x,2)+pow(cfg2.y-st_now->y,2)+pow(cfg2.z-st_now->z,2));
	   v_end<< cfg2.x-st_now->x<< cfg2.y-st_now->y<< cfg2.z-st_now->z;  
	    
	   if( dot(u,v_end)<=0&& end_dis<= 3*checkparas_pt->end_r || end_dis<= checkparas_pt->end_r
	     ||dot(u,v_end)<=0 && *sub_length_pt > s_end-s_init
	     ||*sub_length_pt > 3*(s_end-s_init) 
	     ) 
	     break;

	}//while ends
        //std::cout<<"st_now: "<< st_now->x << " "<< st_now->y <<" "<< st_now->z<< std::endl; 
	//st_final= st_now->copy();
	*st_final= *st_now;
	//std::cout<<"st_final: "<<st_final->x<<" "<<st_final->y<<" "<<st_final->z << std::endl;
	delete st_now;
        if(if_colli) result= -1;
        return result;

     }//DubinsSubCheck ends

};//namespace ends
