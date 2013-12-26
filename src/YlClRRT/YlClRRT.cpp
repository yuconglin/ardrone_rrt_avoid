#include "YlClRRT.hpp"
//utilities
#include "NotInRadius.h"
#include "DubinsTotalCheck.h"
//#include "systemtime.h"
//dubins
#include "dubins.h"
//user types
#include "UavBehavior/GeneralBehavior.hpp"
#include "UavConfig/GeneralConfig.h"
#include "UavState/GeneralState.h"
#include "UavState/GSnode.h"
#include "SpaceLimit.h"
//#include "Sampler3D/Sampler3D.hpp"
#include "Sampler3Da/Sampler3Da.hpp"
#include "checkParas.h"
//ros
#include "ros/ros.h"
//std lib
#include <cmath>
//boost
#include <boost/random.hpp>
//tree
#include "tree.hh"
//quad related
#include "QuadCfg.h"
#include "quadDubins3D.h"

using namespace user_types;
using namespace std;

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
     //sample_node.free_point();
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
     TREEIter root_it = main_tree.insert( main_tree.begin(), root_node.copy() );
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

   void YlClRRT::SetSampler( Sampler3Da* _sampler_pt)
   {
     this->sampler_pt= _sampler_pt;
   }

   void YlClRRT::SetObs2D(const std::vector<user_types::obstacle2D>& _obs2ds)
   {
     obs_collect.obs_2ds= _obs2ds;
   }//SetObs2D ends

   void YlClRRT::SetObs3D(const std::vector<user_types::obstacle3D>& _obs3ds)
   {
     obs_collect.obs_3ds= _obs3ds;
   }//SetObs3D ends

   void YlClRRT::SetSampleParas(double _width,double _height)
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
     //sampler_pt->SetParams(x_root, y_root, z_root, x_goal, y_goal, z_goal);
     sampler_pt->SetParams(root_node.state_pt,goal_node.state_pt,spaceLimit_pt,_width,_height);
     //sampler_pt->SetSigmaGa( config_pt->MaxAscend()*0.25 );
     if_sampler_para_set= true;
   }//SetSampleParas()

   void YlClRRT::ClearToDefault()
   {
     //clear vectors
     temp_log.clear();
     traj_rec.clear();
     tree_vector.clear();
     tree_vector_sort.clear();
     goal_connect_nodes.clear();
     dubin_collects.clear();
     path_total.clear();
     //flags to default
     sec_count= 0;
     if_limit_reach= false;
     if_goal_reach= false;
     if_sampler_para_set= false;
   }//ClearToDefault

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
       if(!if_radius) 
       {
	 std::cout<<"in radius"<< std::endl;
	 continue;
       }
       //if too steep
       DubinsPath path;
       double q0[]={x_root,y_root,the_a};
       double q1[]={x_a,y_a,the_a};
       dubins_init(q0,q1,rho,&path);
  
       double length= dubins_path_length(&path);
       double h= fabs(z_a-z_root);
       double gamma_d= atan2(h,length);
       bool if_ga= (gamma_d<= config_pt->MaxAscend() );
       if(!if_ga) 
       {	
	 std::cout<<"ga too large"<< std::endl;
	 continue;
       }//if out of geo fence
       bool if_in= spaceLimit_pt->TellIn(x_a,y_a,z_a);
       if(!if_in) std::cout<<"out fence"<<std::endl;
       else break;
     }//while ends
     //std::cout<<x_a <<" "<<y_a <<" "<<z_a<< " "<< the_a*180./M_PI<< std::endl; 
     //assign to sample node
     sample_node= GSnode( behavior_pt->InitState(x_a,y_a,z_a,0,the_a) );
   }//SampleNode ends
   
   bool YlClRRT::CheckGoalReach(TREEIter it)
   {//true if goal reachable from it
     QuadCfg cfg_start(it->state_pt->x,it->state_pt->y,it->state_pt->z,it->state_pt->yaw);
     QuadCfg cfg_end(goal_node.state_pt->x,goal_node.state_pt->y,goal_node.state_pt->z,goal_node.state_pt->yaw);
     //create a dubins curve connecting the node to the goal
     quadDubins3D dubin_3d(cfg_start,cfg_end,config_pt->rho);
     //check for rise limit
     if( asin( fabs(goal_node.state_pt->z -it->state_pt->z)
	/(dubin_3d.GetHeuriLength()) ) >= config_pt->MaxAscend() )
     {
       //std::cout<<"goal too steep"<<std::endl;
       return false;
     }
     //check for collision
     user_types::GeneralState* st_final= it->state_pt->copy();
     double actual_length= 0.;
     //std::vector<user_types::GeneralState*> path_log;
     int colli= utils::DubinsTotalCheck(dubin_3d,//the dubins curve
                         it->state_pt,//initial actual state
			 st_final,//final state
			 cfg_end,//stop quad state
			 obs_collect,
                         checkparas_pt,
			 config_pt,
			 spaceLimit_pt,
			 0,//path for log
			 &actual_length//actual length tranversed
			 );
     delete st_final;
     //cout<<"CheckGoalReach: colli "<< colli<<endl; 
     if(colli== 1)//free of collision
     {
       it->cost2go= actual_length;
       it->goal_reach= true;
       if(!if_goal_reach) if_goal_reach= true;
       return true;
     }//if colli ends
     return false;
   }//CheckGoalReach ends

   void YlClRRT::PrintPath(const char* pFilename)
   {
     std::ofstream myfile(pFilename);
     if(myfile.is_open())
     {
       for(int i=0;i!=traj_rec.size();++i)
       {
	 user_types::GeneralState* st= traj_rec[i];
	 myfile<< st->x <<" "<<st->y<<" "<<st->z<<" "<<st->t<< endl;
       }
     }//if ends
     myfile.close();
   }//PrintPath ends

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
	//cout<<"sample_raw: "<< sample_raw<< endl;
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
	   GeneralState* start = tree_it->state_pt;
           GeneralState* st_final= start->copy();

	   QuadCfg cfg_start(start->x,start->y,start->z,start->yaw);
	   //if(sample_count==0)
           //  cout<<"expand first: "<< start->x<<" "<<start->y<<" "<<start->z<< endl;

	   QuadCfg cfg_end(sample_node.state_pt->x,sample_node.state_pt->y,sample_node.state_pt->z,sample_node.state_pt->yaw);
	   //create a dubins curve connecting the node to the sampling node
	   quadDubins3D dubin_3d(cfg_start,cfg_end,config_pt->rho);

	   if(asin( fabs(sample_node.state_pt->z-start->z)/(dubin_3d.GetHeuriLength()) ) >= config_pt->MaxAscend() )
	   {
	      cout<<"too steep"<<endl;
	      continue;
	   }
	   else
	   {	      
	      double c_length= 0.;
	      int colli= utils::DubinsTotalCheck(dubin_3d,start,st_final,cfg_end,obs_collect,checkparas_pt,config_pt,spaceLimit_pt,&temp_log,&c_length);

	      if(colli!=-1)
	      {
		++sample_count;
		if( temp_log.size()> 10)
		  InsertDubinsNode( tree_it ); 
		TempLogClear();
		dubin_collects.push_back(dubin_3d);
		//cout << "sample 1 genertated" <<endl;
		delete st_final;
		break;
	      } //if check ends
	      else
		cout<<"sample collision"<< endl;
	   } //else ends
	   TempLogClear();
           delete st_final;
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
	   delete sample_node.state_pt;
	   break;
        }
        //free sample_node
        delete sample_node.state_pt;	
     }//while ends
     TempLogClear(); 
   }//ExpandTree() ends
  
   void YlClRRT::ClearTree()
   {
     for(TREEIter it=main_tree.begin();it!=main_tree.end();++it)
     {//free nodes pointers
        delete(it->state_pt);
     }//
     if(main_tree.size()!=0 )
       main_tree.erase(main_tree.begin() );
   }//ClearTree() ends

   bool GoalCompFunc(TREEIter it1, TREEIter it2)
   {
      return (it1->cost+it1->cost2go) < (it2->cost+it2->cost2go);
   }

   bool YlClRRT::PathGen()
   {
      t_start= ros::Time::now();
      bool if_path= false;
      
      goal_connect_nodes.clear();
      for(TREEIter it = main_tree.begin();it!= main_tree.end();++it)
      {
	if(it->goal_reach)
	  goal_connect_nodes.push_back(it);
      }

      path_total.clear();
      if(!goal_connect_nodes.empty() )		    
      {
	cout<<"goal nodes size="<<" "<<goal_connect_nodes.size()<<endl;
	//sort goal nodes
	std::sort(goal_connect_nodes.begin(),goal_connect_nodes.end(),GoalCompFunc);
	//add the goal to the tree
	cout << "goal: "<<goal_node.state_pt->x <<" "<<goal_node.state_pt->y <<" "<<goal_node.state_pt->z <<endl;
	TREEIter it_next= goal_connect_nodes[0];
	QuadCfg cfg_start(it_next->state_pt->x,it_next->state_pt->y,it_next->state_pt->z,it_next->state_pt->yaw);
	QuadCfg cfg_end(goal_node.state_pt->x,goal_node.state_pt->y,goal_node.state_pt->z,goal_node.state_pt->yaw);
	//create a dubins curve connecting the node to the sampling node
	quadDubins3D dubin_3d(cfg_start,cfg_end,config_pt->rho);

	dubin_collects.push_back(dubin_3d);
	goal_node.idx_dubin= dubin_collects.size()-1;
	it_next =main_tree.append_child(it_next, goal_node.copy() );
	
	//forming the path		   
	while(1)
	{
	  //GeneralState* st= it_next->state_pt;
	  //cout<<st.x<<" "<<st.y<<" "<<st.z<<" "<<it_next->idx_dubin<<endl;
	  path_total.push_back(it_next);
	  it_next= main_tree.parent(it_next);
	  if( it_next==main_tree.begin() )
	  {
	    path_total.push_back(it_next);
	    //cout<<"first dubin_idx: "<< it_next->idx_dubin<< endl;
	    break;
	  }
	}
	std::reverse( path_total.begin(),path_total.end() );
        cout<<"pathgen first: "<< path_total[0]->state_pt->x<<" "<<path_total[0]->state_pt->y<<" "<<path_total[0]->state_pt->z<< endl;	
	//prune the path
	if( path_total.size()>3 )
	{  
	   cout<<"original path length: "<<path_total.size()<<endl;
	   vector<TREEIter> path_prune;
	   int i_start=0,i_end;
	   path_prune.push_back(path_total[i_start]);
	   while(1)
	   {
	     if(i_start==0) 
	       i_end= path_total.size()-2;//the node prior to the goal
	     else 
	       i_end= path_total.size()-1;
	     
	     TREEIter it_start= path_total[i_start];
	     int i; 
	     GeneralState* st_final= it_start->state_pt->copy();

	     for(i= i_end; i> i_start+1; --i)
	     {
	       TREEIter it_end= path_total[i];
	       QuadCfg cfg_start(it_start->state_pt->x,it_start->state_pt->y,it_start->state_pt->z,it_start->state_pt->yaw);
	       QuadCfg cfg_end(it_end->state_pt->x,it_end->state_pt->y,it_end->state_pt->z,it_end->state_pt->yaw);
	       //create a dubins curve connecting the node to the sampling node
	       quadDubins3D dubin_3d(cfg_start,cfg_end,config_pt->rho);
	       
	       double c_length= 0.;
	       int colli= utils::DubinsTotalCheck(dubin_3d,it_start->state_pt,st_final,cfg_end,obs_collect,checkparas_pt,config_pt,spaceLimit_pt,0,&c_length); 	       
	       if(colli==1)
	       {
		 i_start= i;
		 int _idx_dubin= dubin_collects.size();
                 dubin_collects.push_back(dubin_3d);
		 it_end->idx_dubin= _idx_dubin;
		 path_prune.push_back(it_end);
		 break;
	       }//if colli!= -1 ends

	     }//for int i ends
             delete st_final;

	     if(i== i_start+1) break;
	     if(i_start== path_total.size()-1)
	     {
	       path_total= path_prune;
	       break;
	     }
	     if(i_start==path_total.size()-2)
	     {
	       path_prune.push_back( path_total.back() );
	       path_total= path_prune;
	       break;
	     }
	   }//while ends
       }//prune ends
	
	cout<<"path size="<<" "<<path_total.size()<<endl;
	if_path= true; //path found
      } //if !goal_connect ends
      else
      {
	cout<<"no way to the goal"<<endl;
	if_path= false; //path not found 
      }
      
      ros::Duration du= ros::Time::now()- t_start;
      cout<< "time total: "<< du.toSec() << endl;

      //ProfilerStop();
      return if_path;

   }//PathGen() ends
   
   bool YlClRRT::PathCheck(user_types::GeneralState* st_init, int& it_idx,std::vector<user_types::GeneralState*>& log_rec,bool if_log) 
   {  //false if collision free
      cout<<"*************it is path check***************"<<endl;
      t_start= ros::Time::now();
      
      if(path_total.size()==0){
	try {
	  throw std::runtime_error("PathCheck: path_total empty");
	}
	catch (std::runtime_error &e) {
	  std::cout << "Caught a runtime_error exception: "
		<< e.what () << '\n';
	} 
      }//if ends
      
      if(if_log)
      {
        log_rec.clear();
        GeneralState* st_temp= st_init->copy();
        log_rec.push_back( st_temp );
      }
      //to see which wp it starts from
      double dwp[path_total.size()];//wp 0,1,2,3...path_total.size()-1
      double len_wp[path_total.size()-1];//k points, k-1 segments
      //curve 0,1,2...path_total.size()-2
      int idx= -1;
      double dis_temp= 1e8;
      
      for(int i=0;i< path_total.size();++i)
      {
	GeneralState* st= path_total[i]->state_pt;
	double dis=pow(st->x-st_init->x,2)+pow(st->y-st_init->y,2)+pow(st->z-st_init->z,2);
	dwp[i]= sqrt(dis);
	if(dwp[i]<dis_temp )
	{
	   dis_temp= dwp[i];
	   idx= i;
	}//if dis ends

	if( i< path_total.size()-1 )
	//if(i>0)
	{
	  if(i!=path_total.size()-2)
	    len_wp[i]= path_total[i+1]->idx_length;
	  else
	    len_wp[i]= dubin_collects[path_total[i+1]->idx_dubin].GetHeuriLength();
	}
      }
      
      int idx_sec= -1;
      if(idx!=0 && idx!=path_total.size()-1)
      {
	if( dwp[idx-1]/len_wp[idx-1]< dwp[idx+1]/len_wp[idx] )
	  idx_sec= idx-1;
	else
	  idx_sec= idx;
      }
      else
	idx_sec= idx;
      
      cout<<"idx: "<<idx<<" idx_sec: "<< idx_sec <<endl;
      int colli;
      
      GeneralState *st_cu= st_init->copy(),*st_next= st_init->copy(); 

      for(int i=idx_sec; i!= path_total.size()-1; ++i)
      {
	 TREEIter it_wp= path_total[i+1];
	 vector<user_types::GeneralState*> path_sub;
	 vector<user_types::GeneralState*>* path_pt=0;
	 if(if_log) path_pt= &path_sub;
	 double c_length=0;
	 cout<<"xxxxxxxxxxx,it_wp idx_dubin: "<<it_wp->idx_dubin<<endl;
	 quadDubins3D dubin_3d= dubin_collects[it_wp->idx_dubin];
	 //cout<<"dubin start "<<dubin_3d.cfg_start.x<<" "<<dubin_3d.cfg_start.y<<" "<<dubin_3d.cfg_start.z<<endl;
	 //cout<<"dubin end "<<dubin_3d.cfg_end.x<<" "<<dubin_3d.cfg_end.y<<" "<<dubin_3d.cfg_end.z<<endl;
	 //if_colli= db_3d.PropTotalCheck(st_cu,it_wp->state,st_next,obs_collect);
	 QuadCfg cfg_target(it_wp->state_pt->x,it_wp->state_pt->y,it_wp->state_pt->z,it_wp->state_pt->yaw );
	 colli= utils::DubinsTotalCheck(dubin_3d,st_cu,st_next,cfg_target,obs_collect,checkparas_pt,config_pt,spaceLimit_pt,path_pt,&c_length);
         
	 //cout<<"check colli: "<<colli<<endl;
	 //if(i==idx_sec) 
	 cout<<"ini: "<<dubin_3d.cfg_start.x<<" "<<dubin_3d.cfg_start.y<<" "<<dubin_3d.cfg_start.z<<endl;
	 
	 cout<<"idx "<<i<<" "<<st_next->x <<" "<<st_next->y <<" "<<st_next->z <<" "<< st_next->t <<endl;
	 cout<<"ideal "<<i<<" "<< it_wp->state_pt->x <<" "<<it_wp->state_pt->y<<" "<< it_wp->state_pt->z <<endl;

	 if(colli!=1)
	 {
	   it_idx= i+1;
	   break;

	 }
	 else
	 { //cout<<"collision free."<< endl;
	   it_idx= -1;
	   *st_cu= *st_next;
	   if(if_log)
	    log_rec.insert(log_rec.end(),path_sub.begin(),path_sub.end() );
	 }
      }//for int i end
      delete st_cu;
      delete st_next;

      ros::Duration du= ros::Time::now()- t_start;
      cout<<"colli: "<<colli<<endl;
      cout<< "check time total:+++++++++++++++++++++++ "<< du.toSec() << endl;
      return (colli!=1);

   }//end PathCheck	
   
   bool YlClRRT::PathCheckRepeat( user_types::GeneralState* st_current)
   {//if collided, re-select path. True if path is good
      bool if_colli= false,if_path= false;
      TREEIter it_block;
      ros::Time t_start= ros::Time::now(); 
      while(1)
      {
	if(ros::Time::now()- t_start >= ros::Duration(0.1))
        {
          cout<<"PathCheckRepeat time up"<< endl;
	  if_path= false;
	  break;
	}//for time's sake

	if(!PathGen() ) 
	{//no path to the goal
	  cout<<"no path, stop or local avoidance"<<endl;
	  break;
	}
	int it_idx;
	//cout<<"traj_rec size: "<< traj_rec.size()<< endl;
	if_colli= PathCheck(st_current,it_idx,traj_rec,true);
	
	if(!if_colli) 
	{
	  if_path= true;
	  break;
	}
	else
	{
	  //remove child tree and regenerate path
	  if(it_idx== path_total.size()-1||it_idx==path_total.size()-2)
	  {
            TREEIter it_block= path_total[path_total.size()-2];
	    //it_block->goal_reach= false;
	    ClearSubTree(it_block);
	  }
	  else
	  {
	    TREEIter it_block= path_total[it_idx]; 
	    ClearSubTree(it_block);
	  }
	}//if-else ends
	cout<<"xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx"<<endl;
      }//while ends

      ros::Duration du= ros::Time::now()- t_start;
      cout<< "CheckRepeat time total:+++++++++++++++++++++++ "<< du.toSec() << endl;

      return if_path;
   }//PathCheckRepeat() ends

   //clear a sub tree
   void YlClRRT::ClearSubTree( TREEIter it_sub)
   {
      delete it_sub->state_pt;
      for(TREEIter it= it_sub.begin();it!=it_sub.end();++it)
	delete(it->state_pt);
      main_tree.erase(it_sub);
   }//ClearSubTree ends

   void YlClRRT::InsertDubinsNode( TREEIter start_it)
   {
     if(temp_log.size()==0){
       try {
          throw std::runtime_error ("temp_log size zero");
       }
       catch (std::runtime_error &e) {
          std::cout << "Caught a runtime_error exception: "
                  << e.what () << '\n';
       }
     }
     
     TREEIter insert_it;
     double step= config_pt->speed*config_pt->dt; 
     int N_ITER = floor( checkparas_pt->ds_insert/step );//insert interval
     //set dubin index
     int _idx_dubin= dubin_collects.size();
     //start_it.idx_dubin= _idx_dubin; 

     double seg_len = 0.;
     //std::cout<<"insert path_log size: "<< path_copy.size() << std::endl;
     for(int i=1;i!=temp_log.size();++i)
     {
	seg_len+= sqrt( pow(temp_log[i]->x-temp_log[i-1]->x,2) 
                      + pow(temp_log[i]->y-temp_log[i-1]->y,2)
		      + pow(temp_log[i]->z-temp_log[i-1]->z,2)
	          );
        
	if(!if_limit_reach)
	{
	  sec_count= ros::Time::now().toSec()-t_start.toSec();
	  if( sec_count >= t_limit)
	  {
	    if_limit_reach= true;
	    cout<<"stop1"<<endl;
	    break;
	  }
	}

	if(i% N_ITER==0||i== temp_log.size()-1)
	{
	   if(i== N_ITER) insert_it = start_it;//the first one
	   user_types::GSnode cp_node;
	   cp_node.state_pt= temp_log[i]->copy();
	   cp_node.cost= start_it->cost+ seg_len;
	   cp_node.idx_dubin= _idx_dubin;
	   //cp_node.idx_state= i;
	   cp_node.idx_length= seg_len;

	   insert_it=main_tree.append_child(start_it,cp_node);
	   tree_vector.push_back( insert_it );
	   //delete cp_node.state_pt;
	   //goal connecting test
	   CheckGoalReach(insert_it);
	}// for i%N_ITER ends
     }//for int i ends
            
   }//InsertDubinsNode ends

   user_types::GeneralState* YlClRRT::TimeStateEstimate(double now_t,double Dt)
   {//to estimate the state >from the initial state after time interval Dt and assign it to st
     if( traj_rec.empty() )
     {
       try {
        throw std::runtime_error ("traj_rec empty");
       }
       catch (std::runtime_error &e) {
        std::cout << "Caught a runtime_error exception: "
                  << e.what () << '\n';
       } 
     }
     
     user_types::GeneralState* st_start= traj_rec.front();
     double dt=now_t-st_start->t+ Dt;
     
     if(dt > config_pt->dt*traj_rec.size() )
     {
       user_types::GeneralState* st_end= traj_rec.back();
       double dt_a= dt-st_end->t + st_start->t;
       return st_end->InterPolate(dt_a);  
     }
     else
     {
       user_types::GeneralState* st_close= traj_rec[floor(dt/config_pt->dt)]; 
       //std::cout<<"t close: "<< st_close->t<<" t start: "<<st_start->t << std::endl;
       double dt_a= dt-st_close->t + st_start->t;
       return st_close->InterPolate(dt_a);
     }//if else ends

   }//TimeStateEstimate ends

   void YlClRRT::PathToMsg(ardrone_rrt_avoid::DubinPath_msg& path_msg)
   {
      path_msg.dubin_path.clear();
      //if path contains only one nodes, the quad will just stop and hover
      //int8 type, Quad_msg start,Quad_msg pt_i1,Quad_msg pt_i2,Quad_msg end
      //no path
      if( path_total.size()==0 ) return;
       //there is> path
      for(int i=1; i!=path_total.size(); ++i)
      {
	 TREEIter it= path_total[i];
	 quadDubins3D db_3d= dubin_collects[it->idx_dubin]; 
	 ardrone_rrt_avoid::DubinSeg_msg db_msg;//actually a dubin_seg message
	 if(i== 1) 
	   cout<<"ini to msg: "<<db_3d.cfg_start.x<<" "<<db_3d.cfg_start.y<<" "<<db_3d.cfg_start.z<<endl;

	    //type
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
	    db_msg.stop_pt.x= it->state_pt->x;
	    db_msg.stop_pt.y= it->state_pt->y;
	    db_msg.stop_pt.z= it->state_pt->z;
	 //insert into the path
	 path_msg.dubin_path.push_back(db_msg);
	 
      }//for i ends

   }//PathToMsg ends

   void YlClRRT::TempLogClear()
   {
     for(int i=0;i!= temp_log.size();++i)
     {
        user_types::GeneralState* st_pt= temp_log[i];
	delete st_pt;
     }//for int i ends
     temp_log.clear();
   }//TempLogClear() ends

   double YlClRRT::Heuristics( GSnode& node )
   {//calculate the heuristics from the node to the sample node
//basically the dubins length plus vertical height but if too steep just assign a large heuristic
     double heuri=0;
     double s_x= sample_node.state_pt->x;
     double s_y= sample_node.state_pt->y;
     double s_z= sample_node.state_pt->z;
     double s_yaw= sample_node.state_pt->yaw;
     double n_x= node.state_pt->x;
     double n_y= node.state_pt->y;
     double n_z= node.state_pt->z;
     double n_yaw= node.state_pt->yaw;

     double q0[]= {n_x,n_y,n_yaw};
     double q1[]= {s_x,s_y,s_yaw};
  
     DubinsPath path;
     dubins_init(q0, q1, config_pt->rho, &path);
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
   
   void YlClRRT::CheckGoalIfReach()
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
	  
   }//CheckGoalIfReach() ends
   
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
