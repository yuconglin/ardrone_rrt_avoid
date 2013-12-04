#include "ParrotPlan.hpp"
#include "YlClRRT/YlClRRT.hpp"
//ros related
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int16.h"
//ardrone msg
#include "ardrone_rrt_avoid/DubinPath_msg.h"
#include "ardrone_rrt_avoid/ArdroneState_msg.h"
//ardrone state
#include "UavState/ArdroneState.h"
//utilities
#include "systemtime.h"

using namespace std;
namespace Ardrone_rrt_avoid{

  ParrotPlan::ParrotPlan(YlClRRT* _rrt_pt):rrt_pt(_rrt_pt),if_receive(false),if_new_rec(false),if_reach(0),if_state(false) 
  {
    //publishers and subscribers
    //publishers
    pub_path=nh.advertise<ardrone_rrt_avoid::DubinPath_msg>("path",100);
    pub_if_new=nh.advertise<std_msgs::Bool>("if_new_path",100);
    //subscribers
    sub_receive=nh.subscribe("path_rec",100,&ParrotPlan::receiveCb,this);
    sub_if_new_rec=nh.subscribe("if_new_rec",100,&ParrotPlan::recNewCb,this);
    sub_reach=nh.subscribe("quad_reach",100,&ParrotPlan::reachCb,this);
    sub_state =nh.subscribe("quad_state",100,&ParrotPlan::stateCb,this);    
  }//ParrotPlan ends

  void ParrotPlan::receiveCb(const std_msgs::Bool::ConstPtr& msg)
  {
    if_receive= msg->data;   
  }
  
  void ParrotPlan::recNewCb(const std_msgs::Bool::ConstPtr& msg)
  {
    if_new_rec= msg->data;
  }
  
  void ParrotPlan::reachCb(const std_msgs::Int16::ConstPtr& msg)
  {
    if_reach= msg->data;
  }
  
  void ParrotPlan::stateCb(const ardrone_rrt_avoid::ArdroneState_msg::ConstPtr& msg)
  {
    st_current.x= msg->x;
    st_current.y= msg->y;
    st_current.z= msg->z;
    st_current.yaw= msg->yaw;
    st_current.vx= msg->vx;
    st_current.vy= msg->vy;
    st_current.vz= msg->vz;
    st_current.yaw_rate= msg->yaw_rate;
    st_current.t= msg->t;
    //update if_state flag
    if_state= true;
  }

  int ParrotPlan::PathPlanning()
  {
    ofstream myfile("virtual_replan_rec.txt");
    //messages
    ardrone_rrt_avoid::DubinPath_msg path_msg;
    std_msgs::Bool if_new_msg;

    int case_idx =PATH_CHECK;
    int pre_case_idx= -1;
    bool if_path_good= false;
  
    if_state= false;
    user_types::GeneralState* st_root_pt= NULL;
    user_types::ArdroneState st_check; 
    //ros sleep for msgs to be stable
    ros::Duration(t_offset).sleep();
    rrt_pt->SetStartTime( ros::Time::now() );
    rrt_pt->ExpandTree();
    bool if_first= true;
    //bool if_recheck_start= false;
    t_limit= rrt_pt->GetTimeLimit();

    //while loop
    while(ros::ok())
    {
      if(if_reach==2)
      case_idx= ARRIVED;

      switch(case_idx)
      {
        case PATH_READY:
	{
           if(pre_case_idx!= PATH_READY)
	     cout<<"********PATH READY********"<<endl; 
	   pre_case_idx= case_idx;
	   if(!if_receive)
	   {
	     pub_path.publish(path_msg);
	   } 
	   if(!if_new_rec)
	   {
	     if_new_msg.data= true;
	      pub_if_new.publish(if_new_msg);
	   }
           if(if_receive && if_new_rec)
	   {
	     if(!if_path_good)
	      case_idx= WAIT_STATE;
	   }//if (if_receive) ends

           break;
	}
	case PATH_CHECK:
	{
          if(pre_case_idx!= PATH_CHECK)   
	    cout<<"*****************PATH CHECK***************"<<endl;
	  pre_case_idx= case_idx;
	  if(if_state)//that means an updated quad state is received
	  { 
	    if( st_current.t-st_check.t>=t_limit && if_path_good
	      ||!if_path_good
	      )
	    {
	      if(!if_path_good ) st_check= st_current;

	      if( rrt_pt->PathCheckRepeat(&st_current) )
	      {//a good path is available
		if(st_root_pt) delete st_root_pt;
		st_root_pt= rrt_pt->TimeStateEstimate(t_limit);
		
		//for logging
		if(if_first)
		{
		   if_first= false;
		   vector<user_types::GeneralState*>* traj_pt= rrt_pt->GetTrajRecPt();
		   for(int i=0;i!= traj_pt->size();++i)
		   {
		     user_types::GeneralState* st= traj_pt->at(i);
		      if(myfile.is_open() )
			myfile<< st->x <<" "<<st->y<<" "<<st->z<<" "<<st->t<< endl;
		   }//for int i ends
		}
		if_path_good= true;
	      }
	      else
	      {
		cout<<"plan no path"<<endl;
		if_path_good= false;
	      }

	      //path to msg
	      rrt_pt->PathToMsg(path_msg);
	      //cout<<"path_msg size: "<<path_msg.dubin_path.size()<<endl;
	      case_idx= PATH_READY;
	    }//end if d_t
	    if_state= false;
	  }//end if_state
	  break;
	}
	case TREE_EXPAND:
	{
           if(pre_case_idx!= TREE_EXPAND)
	      cout<<"**********TREE EXPAND**************"<<endl;
	   pre_case_idx= case_idx;
	   rrt_pt->ClearToDefault();
	   rrt_pt->ClearTree();
	   //reset the obstacles
	   //reset tree root
	   rrt_pt->SetRoot(st_root_pt);
	   //reset sample parameters because they are influenced by goal and root
	   rrt_pt->SetSampleParas();
	   //tree expand within time limit
	   rrt_pt->ExpandTree();
	   //tree expanding ends, wait for current quad state and check path
	   case_idx= PATH_CHECK;
	   break;
	}
        case WAIT_STATE:
	{ 
	  if(pre_case_idx!= WAIT_STATE)  
	     cout<<"*************WAIT STATE*****************"<<endl;
	  pre_case_idx= case_idx;
	  if(if_state)
	  {
	    user_types::GeneralState* temp_pt= &st_current;
	    if(st_root_pt) delete st_root_pt;
	    st_root_pt= temp_pt->copy();
	    //st_pre= st_current;
	    case_idx= TREE_EXPAND;
	    if_state= false;
	  }
	  break;
	}
	case ARRIVED:
	{
           if(pre_case_idx!= ARRIVED)
	     cout<<"*****************ARRIVED***********"<< endl;  
	   cout<<"arrived:hohoho"<<endl;
	   pre_case_idx= case_idx;
	   rrt_pt->ClearToDefault();
	   rrt_pt->ClearTree();
	   break;
	}
	default:
	{
           break;
	}
      }//switch ends
      ros::spinOnce();
    }//while ends
    myfile.close();
    return 0;
  }//PathPlanning ends

  int ParrotPlan::working()
  {
    //file for navdata
    //string str_time;
    //char file_nav[256];
    //sprintf( file_nav, "data/%s:%s.txt",str_time.c_str(),"plan");

    ofstream myfile("virtual_replan_rec.txt");
    //ofstream myfile(file_nav);
    //messages
    ardrone_rrt_avoid::DubinPath_msg path_msg;
    std_msgs::Bool if_new_msg;
  
    int case_idx =PATH_CHECK;
    int pre_case_idx= -1;
    bool if_path_good= false;
  
    if_state= false;
    user_types::GeneralState* st_root_pt= NULL;
    //user_types::ArdroneState st_pre;//current quad state and previous quad state
    user_types::ArdroneState st_check, st_recheck;//the state for check
    //ros sleep for msgs to be stable
    ros::Duration(t_offset).sleep();
    rrt_pt->SetStartTime( ros::Time::now() );
    //rrt_pt->SetIfInRos(false);
    //the rest already set outside:root,goal,obstacle,geofence, etc

    bool if_first= true;
    //bool if_recheck_start= false;
    t_limit= rrt_pt->GetTimeLimit();
    //start the process
    rrt_pt->ExpandTree(); 
    //while loop: keep planning and sending
    while(ros::ok() )
    { //if arrived at the goal
      if(if_reach==2)
	case_idx= ARRIVED;

      switch(case_idx)
      {
	 case PATH_READY:
	 {//path ready to send
           if(pre_case_idx!= PATH_READY)
	     cout<<"********PATH READY********"<<endl; 
	   pre_case_idx= case_idx;
	   if(!if_receive)
	   {
	     pub_path.publish(path_msg);
	   } 
	   if(!if_new_rec)
	   {
	     if_new_msg.data= true;
	     pub_if_new.publish(if_new_msg);
	   }
	   //publish if a new path is generated
	   if(if_receive && if_new_rec)
	   {
	     if(if_path_good){
	       //first check if the previous path is still collision free
	       //if_recheck_start= true;
	       st_recheck= st_current;
	       case_idx= PATH_RECHECK; 
	     }//if_path_good ends
	     else
	      case_idx= WAIT_STATE;
	   }//if (if_receive) ends
	   break;
	 }
	 case TREE_EXPAND:
	 { //tree expand to generate a new path
           if(pre_case_idx!= TREE_EXPAND)
	     cout<<"**********TREE EXPAND**************"<<endl;
	   pre_case_idx= case_idx;
	   //clean previously tree and vectors
	   //actually, before tree expand we can check if previously path is still collision free, if so, we can still convert it to a message and ready to send if no new path is available.
	   rrt_pt->ClearToDefault();
	   rrt_pt->ClearTree();
	   //reset the obstacles
	   //reset tree root
	   rrt_pt->SetRoot(st_root_pt);
	   //reset sample parameters because they are influenced by goal and root
	   rrt_pt->SetSampleParas();
	   //tree expand within time limit
	   rrt_pt->ExpandTree();
	   //tree expanding ends, wait for current quad state and check path
	   case_idx= PATH_CHECK;
	   break;
	 }
	 case PATH_CHECK:
	 {
	   if(pre_case_idx!= PATH_CHECK)   
	     cout<<"*****************PATH CHECK***************"<<endl;
	   pre_case_idx= case_idx;
	   if(if_state)//that means an updated quad state is received
	   { 
	     //double d_t= st_current.t-st_pre.t;
	     //st_pre= st_current;
	     //only check when a travelled state is received
	     if( st_current.t-st_check.t>=t_limit && if_path_good
	       ||!if_path_good
	       )
	     {
	       if(!if_path_good ) st_check= st_current;

	       if( rrt_pt->PathCheckRepeat(&st_current) )
	       {//a good path is available
		 if(st_root_pt) delete st_root_pt;
		 st_root_pt= rrt_pt->TimeStateEstimate(t_limit);
		 
		 //for logging
		 if(if_first)
		 {
		    if_first= false;
		    vector<user_types::GeneralState*>* traj_pt= rrt_pt->GetTrajRecPt();
		    for(int i=0;i!= traj_pt->size();++i)
		    {
		      user_types::GeneralState* st= traj_pt->at(i);
		       if(myfile.is_open() )
			 myfile<< st->x <<" "<<st->y<<" "<<st->z<<" "<<st->t<< endl;
		    }//for int i ends
		 }
		 if_path_good= true;
	       }
	       else
	       {
		 cout<<"plan no path"<<endl;
		 if_path_good= false;
	       }

	       //path to msg
	       rrt_pt->PathToMsg(path_msg);
	       //cout<<"path_msg size: "<<path_msg.dubin_path.size()<<endl;
	       case_idx= PATH_READY;
	     }//end if d_t
	     if_state= false;
	   }//end if_state
	   break;
	 }
	 case WAIT_STATE:
	 { 
	   if(pre_case_idx!= WAIT_STATE)  
	      cout<<"*************WAIT STATE*****************"<<endl;
	   pre_case_idx= case_idx;
	   if(if_state)
	   {
	     user_types::GeneralState* temp_pt= &st_current;
	     if(st_root_pt) delete st_root_pt;
	     st_root_pt= temp_pt->copy();
	     //st_pre= st_current;
	     case_idx= TREE_EXPAND;
	     if_state= false;
	   }
	   break;
	 }
	 case PATH_RECHECK:
	 {
	   //check the same path with a predicted state after dt
	   if(pre_case_idx!= PATH_RECHECK)
	     cout<<"*******************PATH RECHECK*************"<<endl;
	   pre_case_idx= case_idx;
	   //sth may happen when it is closet to the last sec
	   if(if_state)
	   {
	     //double d_t= st_current.t-st_pre.t;
	     //st_pre= st_current;
	     user_types::GeneralState* predict_pt= NULL;

	     if(st_current.t-st_recheck.t>= t_limit)
	     {
	       predict_pt= rrt_pt->TimeStateEstimate(t_limit);
	       //reset the obstacles 
	       int idx;
	       std::vector<user_types::GeneralState*> temp_rec;
	       //st_root is actually predicted state
	       if( !rrt_pt->PathCheck(predict_pt,idx,temp_rec,false) )
	       {
		 cout<<"still good path, yeah"<<endl;
		 case_idx= PATH_READY;
		 //st_root needs updating
	       }
	       else
	       {
		 //cout<<"sorry,we need a new path"<<endl;
		 case_idx= TREE_EXPAND;
	       }
	     }//if d_t ends
	     if(predict_pt) delete predict_pt;
	     if_state= false; 
	   }//if_state
	   break;

	 }
	 case ARRIVED:
	 {
	   if(pre_case_idx!= ARRIVED)
	     cout<<"************ARRIVED***********"<<endl;  
	   cout<<"arrived:hohoho"<<endl;
	   pre_case_idx= case_idx;
	   rrt_pt->ClearToDefault();
	   rrt_pt->ClearTree();
	   break;
	 }
	 default:
	 {
	   break;
	 }
      }//switch ends
      //cout<<"once once once"<<endl;
      ros::spinOnce();
    }//while ends
      myfile.close();
      return 0;

  }//working ends

};//namespace ends
