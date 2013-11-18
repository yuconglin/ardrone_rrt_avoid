#include "YlClRRT.hpp"
//utilities
#include "NotInRadius.h"
//dubins
#include "dubins.h"
//user types
#include "UavBehavior/GeneralBehavior.hpp"
#include "UavConfig/GeneralConfig.h"
#include "UavState/GeneralState.h"
#include "SpaceLimit.h"

using namespace user_types;

namespace Ardrone_rrt_avoid{

   YlClRRT::YlClRRT()
   {
     //set user defined types to none
     spaceLimit_pt= NULL;
     config_pt= NULL;
     behavior_pt= NULL;
     //protection flags to default
     if_goal_set= false;
     if_root_set= false;
     if_goal_reach= false;
     if_sampler_para_set= false;
     if_config_set= false;
     if_spacelimit_set= false;

   }//YlClRRT() ends

   /*
   YlClRRT::~YlClRRT()
   {
     if(if_spacelimit_set)
       delete spaceLimit_pt;
     if(if_config_set)
       delete config_pt;
     if(if_behavior_set)
       delete behavior_pt;
     //delete the whole tree's pointer
   }*/  //~YlClRRT() ends  

   void YlClRRT::ConfigFill(const char* pFilename)
   {
     config_pt->ParamfromXML(pFilename);
     if_config_set= true;
   }//ConfigFill ends

   void YlClRRT::SetCheckParas()
   {
      CheckConfigSet();
      checkParas.end_r= max( config_pt->speed*config_pt->dt,0.15 );
      checkParas.ds_check= config_pt->speed*config_pt->dt;
      checkParas.ds_insert= 5*checkParas.ds_check;
   }//CheckParasSet ends

   void YlClRRT::SetGeoFence(user_types::SpaceLimit* _space_pt)
   {
     spaceLimit_pt= _space_pt;
     if_spacelimit_set= true;
   }

   //set root and goal node
   void YlClRRT::SetRoot( GeneralState* state_pt )
   {
     root_node= GSnode(state_pt);
     if_root_set= true;
   }//SetRoot ends
   
   void YlClRRT::SetGoal( GeneralState* state_pt )
   {
     goal_node= GSnode(state_pt);
     if_goal_set= true;
   }//SetGoal ends

   void YlClRRT::SetBehavior( GeneralBehavior* _behavior_pt )
   {
     behavior_pt= _behavior_pt;
     if_behavior_set= true;
   }

   void YlClRRT::SetSampleParas()
   {
     CheckGoalSet();
     CheckRootSet();     
     CheckConfigSet();          
     double x_root= root_node.state_pt->x;
     double y_root= root_node.state_pt->y;
     double z_root= root_node.state_pt->z;
     double x_goal= goal_node.state_pt->x;
     double y_goal= goal_node.state_pt->y;
     double z_goal= goal_node.state_pt->z;
     
     sampler.SetParams(x_root, y_root, z_root, x_goal, y_goal, z_goal);
     sampler.SetSigmaGa( config_pt->MaxAscend()*0.25 );
     if_sampler_para_set= true;
   }//SetSampleParas()

   void YlClRRT::SampleNode()
   {//sample and check
     sampler.SetSampleMethod(1);
          
     double x_root= root_node.state_pt->x;
     double y_root= root_node.state_pt->y;
     double z_root= root_node.state_pt->z;
     double x_a,y_a,z_a,the_a;
     the_a= root_node.state_pt->yaw;
     double rho= config_pt->rho;

     while(1)
     {
       sampler.GetSample(x_a,y_a,z_a,root_node.state_pt,goal_node.state_pt);
       //check
       //if in radius range
       double if_radius= utils::NotInRadius(x_root,y_root,the_a,x_a,y_a,rho);
       if(!if_radius) continue;
       //if too steep
       DubinsPath path;
       double q0[]={x_root,y_root,the_a};
       double q1[]={x_a,y_a,the_a};
       dubins_init(q0,q1,rho,&path);
  
       double length= dubins_path_length(&path);
       double h= abs(z_a-z_root);
       double gamma_d= atan2(h,length);
       bool if_ga= (gamma_d<= config_pt->MaxAscend() );
       if(!if_ga) continue;
       //if out of geo fence
       bool if_in= spaceLimit_pt->TellIn(x_a,y_a,z_a);
       if(if_in) break;
     }//while ends
     std::cout<<x_a <<" "<<y_a <<" "<<z_a<< " "<< the_a*180./M_PI<< std::endl; 
     //assign to sample node
     sample_node= GSnode( behavior_pt->InitState(x_a,y_a,z_a,0,the_a) );
   }//SampleNode ends

   void YlClRRT::CheckGoalSet()
   {
      if(!if_goal_set){
        try {
	  throw std::runtime_error ("goal not set");
        }
        catch (std::runtime_error &e) {
	  std::cout<< "Caught a runtime_error exception: "
	       << e.what () << '\n';
        } 
      }//if_goal_set not

   }//CheckGoalSet() ends
   
   void YlClRRT::CheckRootSet()
   {
      if(!if_root_set){
       try {
	 throw std::runtime_error ("root not set");
       }
       catch (std::runtime_error &e) {
	 std::cout<< "Caught a runtime_error exception: "
		<< e.what () << '\n';
       } 
     }//if_root_set not

   }//CheckRootSet() ends
   
   void YlClRRT::CheckGoalReach()
   {
       if(!if_goal_reach){
	    try {
	      throw std::runtime_error ("goal not reached");
	    }
	    catch (std::runtime_error &e) {
	      std::cout<< "Caught a runtime_error exception: "
		     << e.what () << '\n';
	    } 
	  }//if_root_set not
	  
   }//CheckGoalReach() ends
   
   void YlClRRT::CheckSampleParaSet()
   {
     if(!if_sampler_para_set){
       try {
	 throw std::runtime_error ("sampler paras not set");
       }
       catch (std::runtime_error &e) {
	 std::cout<< "Caught a runtime_error exception: "
		<< e.what () << '\n';
       } 
     }

   }//CheckSampleParaSet() ends
   
   void YlClRRT::CheckConfigSet()
   {
       if(!if_config_set){
	try {
	  throw std::runtime_error ("config not set");
	}
	catch (std::runtime_error &e) {
	  std::cout<< "Caught a runtime_error exception: "
		 << e.what () << '\n';
	} 
      }
  
   }//CheckConfigSet() ends
   
   void YlClRRT::CheckSpaceLimitSet()
   {
       if(!if_spacelimit_set){
	try {
	  throw std::runtime_error ("space limit not set");
	}
	catch (std::runtime_error &e) {
	  std::cout<< "Caught a runtime_error exception: "
		 << e.what () << '\n';
	} 
      }
 
   }//CheckSpaceLimitSet() ends

   void YlClRRT::CheckBehaviorSet()
   {
      if(!if_behavior_set){
       try {
	 throw std::runtime_error ("behavior not set");
       }
       catch (std::runtime_error &e) {
	 std::cout<< "Caught a runtime_error exception: "
		<< e.what () << '\n';
       } 
     }

   }//CheckBehaviorSet() ends

   void YlClRRT::CheckParasSet()
   {
      if(!if_checkparas_set){
        try {
	     throw std::runtime_error ("paras for check dubins not set");
        }
        catch (std::runtime_error &e) {
	   std::cout<< "Caught a runtime_error exception: "
	            << e.what () << '\n';
        } 
      }  

   }//CheckParasSet() ends
   
   void YlClRRT::CheckFlagsSet()
   {
     CheckGoalSet();
     CheckRootSet();
     //CheckGoalReach();
     CheckSampleParaSet();
     CheckConfigSet();
     CheckSpaceLimitSet();
     CheckBehaviorSet();
   }//CheckFlagsSet() ends
};//namespace ends
