#include "ParrotPlan.hpp"
//ros related
#include "std_msgs/Bool.h"
#include "std_msgs/Int16.h"
//ardrone msg
#include "ardrone_rrt_avoid/DubinPath_msg.h"
#include "ardrone_rrt_avoid/QuadState_msg.h"
//ardrone state
#include "UavState/ArdroneState.h"

namespace Ardrone_rrt_avoid{

  ParrotPlan::ParrotPlan(YlClRRT* _rrt_pt):rrt_pt(_rrt_pt),if_receive(false),if_new_rec(false),if_reach(0),if_state(false) 
  {
    //publishers and subscribers
    //publishers
    pub_path=nh.advertise<ardrone_rrt_avoid::DubinPath_msg>("path",100);
    pub_if_new=nh.advertise<std_msgs::Bool>("if_new_path",100);
    //subscribers
    sub_receive=nh.subscribe("path_rec",100,&ReplanNode::receiveCb,this);
    sub_if_new_rec=nh.subscribe("if_new_rec",100,&ReplanNode::recNewCb,this);
    sub_reach=nh.subscribe("quad_reach",100,&ReplanNode::reachCb,this);
    sub_state =nh.subscribe("quad_state",100,&ReplanNode::stateCb,this);    
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
    st_current.t= msg->t;
    //update if_state flag
    if_state= true;
  }

  void ParrotPlan::working()
  {
    ofstream myfile("virtual_replan_rec.txt");
    //messages
    ardrone_rrt_avoid::DubinPath_msg path_msg;
    ardrone_rrt_avoid::Bool if_new_msg;
  
    int case_idx =PATH_CHECK;
    bool if_path_good= false;
  
    if_state= false;
    user_types::ArdroneState st_root;
    user_types::ArdroneState st_pre;//current quad state and previous quad state
    //ros sleep for msgs to be stable
    ros::Duration(t_offset).sleep();
    rrt_pt->SetStartTime( ros::Time::now() );
    //rrt_pt->SetIfInRos(false);
    //the rest already set outside:root,goal,obstacle,geofence, etc

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
	   cout<<"********PATH READY********"<<endl; 
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
	       case_idx= PATH_RECHECK; 
	     }//if_path_good ends
	     else
	      case_idx= WAIT_STATE;
	   }//if (if_receive) ends
	   break;
	 }
	 case TREE_EXPAND:
	 { //tree expand to generate a new path
	   cout<<"**********TREE EXPAND**************"<<endl;
	   //clean previously tree and vectors
	   //actually, before tree expand we can check if previously path is still collision free, if so, we can still convert it to a message and ready to send if no new path is available.
	   rrt_pt->ClearToDefault();
	   rrt_pt->ClearTree();
	   //reset tree root
	   //cout<<"st_root: "<<st_root.x<<","<<st_root.y<<","<<st_root.z<<","\
	       <<st_root.theta<<","<<st_root.v<<","<<st_root.vz<<","\
	       <<st_root.t<<endl;
	   //cout<<"goal: "<<goal_node.state.x<<","<<goal_node.state.y<<","<<goal_node.state.z<<endl;
	   rrt_pt->SetRoot(st_root);
	   //reset sample parameters because they are influenced by goal and root
	   rrt_pt->SetSamplesParas();
	   //tree expand within time limit
	   rrt_pt->ExpandTree();
	   //tree expanding ends, wait for current quad state and check path
	   case_idx= PATH_CHECK;
	   break;
	 }
	 case PATH_CHECK:
	 {
	   cout<<"*****************PATH CHECK***************"<<endl;
	   if(if_state)//that means an updated quad state is received
	   { 
	     //cout<<"st_current: "<<st_current.x<<" "<<st_current.y<<" "\
		 <<st_current.z<<" "<<st_current.theta/M_PI*180<<" "\
		 <<st_current.v<<" "<<st_current.vz<<" "<<st_current.t<<endl;
	     quad.SetCurrentSt( st_current );
	     double d_t= st_current.t-st_pre.t;
	     st_pre= st_current;
	     //only check when a travelled state is received
	     if( d_t>0.5*t_limit && if_path_good
	       ||!if_path_good
	       )
	     {
	       if( quad.PathCheckRepeat() )
	       {//a good path is available
		 quad.TimeStateEstimate(t_limit,st_root);
		 //cout<<"st_root estimate: "<<st_root.x<<" "<<st_root.y<<" "\
		     <<st_root.z<<" "\
		     <<st_root.theta<<" "<<st_root.v<<" "<<st_root.vz<<" "\
		     <<st_root.t<<endl;

		 //for logging
		 if(if_first)
		 {
		    if_first= false;
		    vector<QuadState>* real_pt= quad.GetTrajRec();
		    for(int i=0;i!= real_pt->size();++i)
		    {
		       //if(i*quad.GetDt() > t_limit)
		       //  break;
		       QuadState st= real_pt->at(i);
		       if(myfile.is_open() )
			 myfile<< st.x<<" "<<st.y<<" "<<st.z<<" "<<st.t<< endl;
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
	       quad.PathToMsg(path_msg);
	       //cout<<"path_msg size: "<<path_msg.dubin_path.size()<<endl;
	       case_idx= PATH_READY;
	     }//end if d_t
	     if_state= false;
	   }//end if_state
	   break;
	 }
	 case WAIT_STATE:
	 { 
	   cout<<"*************WAIT STATE*****************"<<endl;
	   if(if_state)
	   {
	     st_root= st_current;
	     st_pre= st_current;
	     case_idx= TREE_EXPAND;
	     if_state= false;
	   }
	   break;
	 }
	 case PATH_RECHECK:
	 {
	   //check the same path with a predicted state after dt
	   cout<<"*******************PATH RECHECK*************"<<endl;
	   //sth may happen when it is closet to the last sec
	   if(if_state)
	   {
	     //cout<<"recheck st_current: "<<st_current.x<<" "<<st_current.y<<" "\
		 <<st_current.z<<" "<<st_current.theta/M_PI*180<<" "\
		 <<st_current.v<<" "<<st_current.vz<<" "<<st_current.t<<endl;
	     quad.SetCurrentSt( st_current );
	     double d_t= st_current.t-st_pre.t;
	     st_pre= st_current;
	     QuadState predict_st;

	     if(d_t>0.5*t_limit)
	     {
	       quad.TimeStateEstimate(t_limit,predict_st);
	       if(which_obs==1)
	       {
		 //reset the obstacles
		 vector<obstacle3D> obs_vec;
		 obstacle3D obs;
		 ob_log.t_obstacle( predict_st.t,obs,ob_idx );
		 obs_vec.clear();
		 obs_vec.push_back(obs);
		 quad.SetObs(obs_vec);
	       }
	       TREEIter it_block;
	       vector<QuadState> temp_rec;
	       //st_root is actually predicted state
	       if( !quad.PathCheck(predict_st,it_block,temp_rec) )
	       {
		 //cout<<"still good path, yeah"<<endl;
		 case_idx= PATH_READY;
		 //st_root needs updating
	       }
	       else
	       {
		 //cout<<"sorry,we need a new path"<<endl;
		 case_idx= TREE_EXPAND;
	       }
	     }//if d_t ends
	     if_state= false; 
	   }//if_state
	   break;

	 }
	 case ARRIVED:
	 {
	   cout<<"arrived:hohoho"<<endl;
	   break;
	 }
	 default:
	 {
	   break;
	 }
      }//switch ends
      //cout<<"once once once"<<endl;
      ros::spinOnce();

  }//working ends

};//namespace ends
