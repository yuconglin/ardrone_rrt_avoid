#include "YlClRRT.hpp"
//utilities
#include "NotInRadius.h"
//dubins
#include "dubins.h"
//user types
#include "UavBehavior/GeneralBehavior.hpp"
#include "UavConfig/GeneralConfig.h"
#include "UavState/GeneralState.h"
#include "UavState/GSnode.h"
#include "SpaceLimit.h"
#include "Sampler3D/Sampler3D.hpp"
#include "checkParas.h"
//ros
#include "ros/ros.h"
//std lib
#include <cmath>
//tree
#include "tree.hh"

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
     if_behavior_set= false;
     if_checkparas_set= false;
     if_in_ros= false;
     if_limit_reach= false;
     sec_count= 0.;
   }//YlClRRT() ends
 
   YlClRRT::~YlClRRT()
   {
     if(if_spacelimit_set)
       delete spaceLimit_pt;
     if(if_config_set)
       delete config_pt;
     if(if_behavior_set)
       delete behavior_pt;
     if(if_sampler_para_set)
       delete sampler_pt;
     if(if_checkparas_set)
       delete checkparas_pt;
     //root and goal
     goal_node.free_point();
     root_node.free_point();
     sample_node.free_point();
     //delete the whole tree's pointer
   }  //~YlClRRT() ends  

   void YlClRRT::ConfigFill(const char* pFilename)
   {
     config_pt->ParamfromXML(pFilename);
     if_config_set= true;
   }//ConfigFill ends

   void YlClRRT::SetCheckParas()
   {
      CheckConfigSet();
      checkparas_pt= new checkParas();
      checkparas_pt->end_r= std::max( config_pt->speed*config_pt->dt,0.15 );
      checkparas_pt->ds_check= config_pt->speed*config_pt->dt;
      checkparas_pt->ds_insert= 5*checkparas_pt->ds_check;
      if_checkparas_set= true;
   }//CheckParasSet ends

   void YlClRRT::SetGeoFence(user_types::SpaceLimit* _space_pt)
   {
     spaceLimit_pt= _space_pt;
     if_spacelimit_set= true;
   }
   
   double YlClRRT::GetRho()
   {
     return config_pt->rho;
   } 
   
   //set root and goal node
   void YlClRRT::SetRoot( GeneralState* state_pt )
   {
     root_node= GSnode(state_pt);
     TREEIter root_it = main_tree.insert( main_tree.begin(), root_node );
      //push root iter to the vector
     tree_vector.push_back(root_it);
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

   void YlClRRT::SetSampler( Sampler3D* _sampler_pt)
   {
     this->sampler_pt= _sampler_pt;
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
     
     sampler_pt->SetSampleMethod(1);
     sampler_pt->SetParams(x_root, y_root, z_root, x_goal, y_goal, z_goal);
     sampler_pt->SetSigmaGa( config_pt->MaxAscend()*0.25 );
     if_sampler_para_set= true;
   }//SetSampleParas()

   void YlClRRT::SampleNode()
   {//sample and check
     //sampler_pt->SetSampleMethod(1);   
     double x_root= root_node.state_pt->x;
     double y_root= root_node.state_pt->y;
     double z_root= root_node.state_pt->z;
     double x_a,y_a,z_a,the_a;
     the_a= root_node.state_pt->yaw;
     double rho= config_pt->rho;

     while(1)
     {
       sampler_pt->GetSample(x_a,y_a,z_a,root_node.state_pt,goal_node.state_pt);
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
       double h= fabs(z_a-z_root);
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
   
   bool YlClRRT::CheckGoalReach(TREEIter it)
   {//true if goal reachable from it
     QuadCfg cfg_start(it->state_pt->x,it->state_pt->y,it->state_pt->z,it->state_pt->yaw);
     QuadCfg cfg_end(goal_node->state_pt->x,goal_node.state_pt->y,goal_node.state_pt->z,goal_node->state_pt->yaw);
     //create a dubins curve connecting the node to the goal
     quadDubins3D dubin_3d(cfg_start,cfg_end,config_pt->rho);
     //check for rise limit
     if( asin( fabs(goal_node->state_pt->z -it->state_pt->z)
	/(dubin_3d.GetHeuriLength()) ) >= config_pt->MaxAscend() )
     {
       //std::cout<<"goal too steep"<<std::endl;
       return false;
     }
     //check for collision
     user_types::GeneralState* st_final= it->state_pt->copy();
     double actual_length= 0.;
     //std::vector<user_types::GeneralState*> path_log;
     int colli= DubinsTotalCheck(dubin_3d,//the dubins curve
                         it->state_pt,//initial actual state
			 st_final,//final state
			 cfg_end,//stop quad state
			 obstacles,
                         checkparas_pt,
			 config_pt,
			 0,//path for log
			 &actual_length//actual length tranversed
			 );
     delete st_final;
     
     if(colli== 1)//free of collision
     {
       it->cost2go= actual_length;
       it->goal_reach= true;
       if(!if_goal_reach) if_goal_reach= true;
       return true;
     }//if colli ends
     return false;
   }//CheckGoalReach ends

   void YlClRRT::ExpandTree()
   {
     if(!if_in_ros)
     {
       ros::Time::init();
       t_start = ros::Time::now();//timing start point
     }
     cout<<"tree expand starts"<<endl;
     int sample_count = 0;//effective sample
     int sample_raw= 0;//raw samples
     CheckGoalReach( main_tree.begin());
     
     //main loop starts here
     while(1)
     {
        SampleNode();
	++sample_raw;
        CalHeuri();  
	SortNodes();

        for(int j=0;j!=tree_vector_sort.size();++j)
	{
	   if(!if_limit_reach)
	   {
	     sec_count= ros::Time::now().toSec()-t_start.toSec();
	     if( sec_count >= t_limit )
	     {
	       if_limit_reach= true;
	       cout<<"stop2"<<endl;
	       break;
	     }//
	   }
	   
	   TREEIter tree_it = tree_vector_sort[j];
	   GeneralState* start = (*tree_it)->state_pt;
           GeneralState* st_final= start->copy();

	   QuadCfg cfg_start(start->x,start->y,start->z,start->yaw);
	   QuadCfg cfg_end(sample_node->state_pt->x,sample_node->state_pt->y,sample_node->state_pt->z,sample_node->state_pt->yaw);
	   //create a dubins curve connecting the node to the sampling node
	   quadDubins3D dubin_3d(cfg_start,cfg_end,config_pt->rho);

	   if(asin( fabs(sample_node->state_pt->z-start->z)/(dubin_3d.GetHeuriLength()) ) >= config_pt->MaxAscend() )
	   {
	      //cout<<"too steep"<<endl;
	      continue;
	   }
	   else
	   {	      
	      double c_length= 0.;
	      int colli= DubinsTotalCheck(dubin_3d,start,st_final,cfg_end,obstacles,checkparas_pt,config_pt,temp_log,&c_length);

	      if(colli!=-1)
	      {
		++sample_count;
		if( temp_log.size()> 10)
		  InsertDubinsNode( tree_it ); 
		//path_log.clear();
		dubin_collects.push_back(dubin_3d);
		//cout << "sample 1 genertated" <<endl;
		break;
	      } //if check ends
	   } //else ends

        }// for int j ends
        //timer 
        if(!if_limit_reach)
        {
	  sec_count= ros::Time::now().toSec()-t_start.toSec();
	  if( sec_count >= t_limit )
	  {
	    if_limit_reach= true;
	    cout<<"stop3"<<endl;
	   //break;
	  }// 
        }
      
        if(if_limit_reach)
        //while time limit for tree expanding reached 
        {
	   cout<<"time="<<" "<<sec_count<<" "\
	       <<"nodes="<<" "<<tree_vector.size()<<" "\
	       <<"samples good="<<" "<<sample_count<<" "\
	       <<"smaples="<<" "<< sample_raw <<endl;
	   break;
        }

     }//while ends
   
   }//ExpandTree() ends
   
   void YlClRRT::InsertDubinsNode( TREEIter it)
   {
              
   }//InsertDubinsNode ends

   double YlClRRT::Heuristics( GSnode& node )
   {//calculate the heuristics from the node to the sample node
//basically the dubins length plus vertical height but if too steep just assign a large heuristic
     double heuri=0;
     double s_x= sample_node->state_pt->x;
     double s_y= sample_node->state_pt->y;
     double s_z= sample_node->state_pt->z;
     double s_yaw= sample_node->state_pt->yaw;
     double n_x= node->state_pt->x;
     double n_y= node->state_pt->y;
     double n_z= node->state_pt->z;
     double n_yaw= node->state_pt->yaw;

     double q0[]= {n_x,n_y,n_yaw};
     double q1[]= {s_x,s_y,s_yaw};
  
     DubinsPath path;
     dubins_init(q0, q1, rho, &path);
     double length= dubins_path_length(&path);
     double h= fabs(s_z-n_z);
     double gamma_d= atan2(h,length);
     if( gamma_d > config_pt->MaxAscend() ) 
        heuri= 1e8;
     else
     {
        heuri= sqrt(length*length + h*h);
        boost::mt19937 generator;
        generator.seed(static_cast<unsigned int>(std::time(0)));
      
        boost::uniform_real<double> distribution(0.0,1.0);
        boost::variate_generator<boost::mt19937&,boost::uniform_real<> > p_dis(generator, distribution);

        double p= p_dis();
        if( !if_goal_reach && p<=0.3 || if_goal_reach && p>0.3 )
	  heuri= node.cost+ heuri/config_pt->speed;
     }
     return heuri;
}//Heuristics ends
 
   void YlClRRT::CalHeuri()
   {
     for(TREEIter it=main_tree.begin();it!=main_tree.end();++it)
        it->heuri=Heuristics(*it);	          
   }

   bool NodeCompFunc(TREEIter it1,TREEIter it2)
   {
     return it1->heuri < it1->heuri;
   }

   void YlClRRT::SortNodes( )
   {
     tree_vector_sort.clear();
     tree_vector_sort = tree_vector;
     std::sort(tree_vector_sort.begin(), tree_vector_sort.end(), NodeCompFunc);
   }

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
