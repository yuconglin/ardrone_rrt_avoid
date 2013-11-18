#include "DubinsTotalCheck.h"
//quad releated
#include "quadDubins3D.h"
//utils
#include "ColliCheck.h"
//user defined types
#include "GeneralState.h"

namespace utils{

   int DubinsTotalCheck(quadDubins3D& db_3d,//the dubins curve
                         user_types::GeneralState* st_init,//initial actual state
			 user_types::GeneralState* st_final,//final state
			 QuadCfg cfg_target,//stop quad state
			 const vector<obstacle3D>& obstacles;
                         const user_types::checkParas check_paras;
			 user_types::GeneralConfig* config_pt;
			 std::vector<user_types::GeneralState*> path_log,//path for log
			 double& actual_length//actual length tranversed
			 )
   {  //don't forget to delete if needed 
      if(sqrt(pow(st_init->x-cfg_target.x,2)+pow(st_init->y-cfg_target.y,2)+pow(st_init->z-cfg_target.z,2))< check_paras.end_r )
      {
         std::cout <<"already there"<<std::endl;
         st_next= st_init.copy();
         return 1;
      }
      //set actual_length and path_log to default
      actual_length= 0.;
      path_log.clear();
      //see which segment st_init is closest to
      QuadCfg cfgs[]={db_3d.cfg_start,db_3d.cfg_i1,db_3d.cfg_i2,db_3d.cfg_end};
      double d[4];
      int idx= -1;
      double dis_temp = 1e8;
      for(int i=0;i<4;++i)
      {
	  double dis= pow(cfgs[i].x-st_init.x,2)+pow(cfgs[i].y-st_init.y,2)+pow(cfgs[i].z-st_init.z,2);
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
	    throw std::runtime_error ("a runtime error");
	  }
	  catch (std::runtime_error &e) {
	    std::cout << "Caught a runtime_error exception: "
		      << e.what () << '\n';
	  }
      }
      //check ends
      //collision check for each segment of the dubins curve
      int result, colli= 1;
      user_types::GeneralState* st_first= st_init, st_next;
 
      for(int i= idx_seg;i!= 3;++i)
      {
         vector<GeneralState*> path_sub;
	 double length_sub= 0.;
         result= DubinsSubCheck(db_3d,st_first,st_next,cfg_target,obstacles,check_paras,config_pt,path_sub,length_sub,i);
      }//for int i ends

   }//DubinsTotalCheck ends

};//namespace ends
