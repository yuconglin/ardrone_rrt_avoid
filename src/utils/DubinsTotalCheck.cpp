#include "DubinsTotalCheck.h"
#include "DubinsSubCheck.h"
//quad releated
#include "quadDubins3D.h"
#include "QuadCfg.h"
//utils
#include "SingleCheck.h"
//user defined types
#include "UavState/GeneralState.h"
#include "ObsCollect.h"
#include "checkParas.h"
#include "UavConfig/GeneralConfig.h"

using namespace std;
namespace utils{

   int DubinsTotalCheck(quadDubins3D& db_3d,//the dubins curve
                         user_types::GeneralState* st_init,//initial actual state
			 user_types::GeneralState* st_final,//final state
			 QuadCfg& cfg_target,//stop quad state
			 user_types::ObsCollect& obs_collect,
                         user_types::checkParas* checkparas_pt,
			 user_types::GeneralConfig* config_pt,
			 std::vector<user_types::GeneralState*>* path_log_pt,//path for log
			 double* actual_length_pt//actual length tranversed
			 )
   {  //don't forget to delete if needed 
      if(sqrt(pow(st_init->x-cfg_target.x,2)+pow(st_init->y-cfg_target.y,2)+pow(st_init->z-cfg_target.z,2))< checkparas_pt->end_r )
      {
         std::cout <<"already there"<<std::endl;
         st_final= st_init;
         return 1;
      }
      //set actual_length and path_log to default
      *actual_length_pt= 0.;
      if(path_log_pt) path_log_pt->clear();
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
      //cout<<"idx: "<< idx<< endl; 
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
      //std::cout<<"idx_seg: "<< idx_seg<< std::endl;
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
      user_types::GeneralState* st_first= st_init->copy(), *st_next= st_init->copy();
 
      for(int i= idx_seg;i!= 3;++i)
      {
	 std::vector<user_types::GeneralState*>* path_sub_pt= 0;
	 std::vector<user_types::GeneralState*> path_sub;
         if(path_log_pt) path_sub_pt= &path_sub;
	 //std::cout<<"i: "<< i<< std::endl; 
	 double length_sub= 0;
         double *length_sub_pt= &length_sub;
	 result= DubinsSubCheck(db_3d,st_first,st_next,cfg_target,obs_collect,checkparas_pt,config_pt,path_sub_pt,length_sub_pt,i);
	 //std::cout<<"la la la"<< std::endl;
         //modify total length
	 *actual_length_pt+= *length_sub_pt;
	 //update logged path
	 //std::cout<< "path_sub size: "<< path_sub.size()<< std::endl;
	 if(path_log_pt)
	   path_log_pt->insert( path_log_pt->end(),path_sub_pt->begin(),path_sub_pt->end() ); 
         
	 if(result==-1)
         {//collision
           if( *actual_length_pt>= 0.5*(db_3d.seg_param[0]+db_3d.seg_param[1]+db_3d.seg_param[2]) )
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
	   //st_first= st_next->copy();
	   *st_first= *st_next;
         }
         else {;}//nothing
      
         //std::cout<< "st_next: "<<st_next->x <<" "<< st_next->y << " " <<st_next->z << std::endl;

      }//for int i ends
      
      *st_final= *st_next;
      delete st_first;
      delete st_next;
      return colli;
   }//DubinsTotalCheck ends

};//namespace ends
