#include "DubinsTotalCheck.h"
#include "DubinsSubCheck.h"
//quad releated
#include "quadDubins3D.h"
#include "QuadCfg.h"
//utils
#include "SingleCheck.h"
//user defined types
#include "UavState/GeneralState.h"
#include "obstacle3D.h"
#include "checkParas.h"
#include "UavConfig/GeneralConfig.h"

namespace utils{

   int DubinsTotalCheck(quadDubins3D& db_3d,//the dubins curve
                         user_types::GeneralState* st_init,//initial actual state
			 user_types::GeneralState* st_final,//final state
			 QuadCfg& cfg_target,//stop quad state
			 std::vector<user_types::obstacle3D>& obstacles,
                         user_types::checkParas* checkparas_pt,
			 user_types::GeneralConfig* config_pt,
			 std::vector<user_types::GeneralState*> path_log,//path for log
			 double& actual_length//actual length tranversed
			 )
   {  //don't forget to delete if needed 
      if(sqrt(pow(st_init->x-cfg_target.x,2)+pow(st_init->y-cfg_target.y,2)+pow(st_init->z-cfg_target.z,2))< checkparas_pt->end_r )
      {
         std::cout <<"already there"<<std::endl;
         st_final= st_init;
         return 1;
      }
      //set actual_length and path_log to default
      actual_length= 0.;
      path_log.clear();
      //see which segment st_init->is closest to
      QuadCfg cfgs[]={db_3d.cfg_start,db_3d.cfg_i1,db_3d.cfg_i2,db_3d.cfg_end};
      double d[4];
      int idx= -1;
      double dis_temp = 1e8;
      for(int i=0;i<4;++i)
      {
	  double dis= pow(cfgs[i].x-st_init->x,2)+pow(cfgs[i].y-st_init->y,2)+pow(cfgs[i].z-st_init->z,2);
	  d[i]= sqrt(dis);
	  
	  if( db_3d.seg_param[i]==0) 
	  {
	    continue;
	  }

	  if(d[i]<dis_temp)
	  {
	    dis_temp= d[i];
	    idx= i;
	  }//if dis ends
      }//for int i ends
     
      int idx_seg= -1;
      if(idx==1||idx==2)
      {
	  if( d[idx-1]/(db_3d.seg_param[idx-1]+1e-10)<d[idx+1]/(db_3d.seg_param[idx]+1e-10) )
	    idx_seg= idx-1;
	  else
	    idx_seg= idx;
      }
      else
	  idx_seg= idx;//if idx ends
      if(idx_seg== 4) --idx_seg;
      //so idx_seg is the closest segment
      //check
      if(!(idx_seg==0||idx_seg==1||idx_seg==2) ){
	  try {
	    throw std::runtime_error ("idx_seg should be 0,1 or 2");
	  }
	  catch (std::runtime_error &e) {
	    std::cout << "Caught a runtime_error exception: "
		      << e.what () << '\n';
	  }
      }
      //check ends
      //collision check for each segment of the dubins curve
      int result, colli= 1;
      user_types::GeneralState* st_first= st_init->copy();
      user_types::GeneralState* st_next= st_init->copy();
 
      for(int i= idx_seg;i!= 3;++i)
      {
	 std::vector<user_types::GeneralState*> path_sub;
	 double length_sub= 0.;
         result= DubinsSubCheck(db_3d,st_first,st_next,cfg_target,obstacles,checkparas_pt,config_pt,path_sub,length_sub,i);
         //modify total length
	 actual_length+= length_sub;
	 //update logged path
	 path_log.insert( path_log.end(),path_sub.begin(),path_sub.end() ); 

	 if(result==-1)
         {//collision
         if( actual_length>= 0.5*(db_3d.seg_param[0]+db_3d.seg_param[1]+db_3d.seg_param[2]) )
	    colli= 0;
	 else
	    colli= -1;
	 break;
         }
         else if(result== 1)
         {//target reached without collision
           break;
         }
         else if(result== 0)//target not reached,no collision
         {
	   std::cout<< "st_next: "<<st_next->x <<" "<< st_next->y << " " <<st_next->z << std::endl;
	   st_first= st_next->copy();
         }
         else {;}//nothing

      }//for int i ends
      
      st_final= st_next->copy();
      delete st_next;
      delete st_first;
      return colli;
   }//DubinsTotalCheck ends

};//namespace ends
