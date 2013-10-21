#include "VirtualQuad.hpp"
#include "ReplanNode.hpp"
#include "boost/bind.hpp"
#include "quadNode.h"
#include "ObsLog.h"
#include "QuadState.h"
#include "ticpp.h"

ReplanNode::ReplanNode(VirtualQuad& _quad,int _which_obs):quad(_quad),which_obs(_which_obs),if_receive(false),if_new_rec(false),if_reach(0),if_state(false)
{ }//ReplanNode constructor ends
//callback funtions

void ReplanNode::receiveCb(const yucong_rrt_avoid::IfSend_msg::ConstPtr& msg)
{
  if_receive= msg->if_receive;
}

void ReplanNode::recNewCb(const yucong_rrt_avoid::IfNewRec_msg::ConstPtr& msg)
{
  if_new_rec= msg->if_new_rec;  
}

void ReplanNode::reachCb(const yucong_rrt_avoid::IfReach_msg::ConstPtr& msg)
{
  if_reach= msg->if_reach;
}


void ReplanNode::stateCb(const yucong_rrt_avoid::QuadState_msg::ConstPtr& msg)
{
  st_current.x= msg->x;
  st_current.y= msg->y;
  st_current.z= msg->z;
  st_current.theta= msg->theta;
  st_current.v= msg->v;
  st_current.vz= msg->vz;
  st_current.t= msg->t;
  //update if_state flag
  if_state= true;
  //cout<<"if_state: "<< if_state<<endl;
}

int ReplanNode::SetNormalObs()
{
  quadNode root_nd= quad.GetRoot();
  quadNode goal_nd= quad.GetGoal();
  //read distance and velocity of the obstacle from param.xml
  double lambda,ob_speed;
  try
  {
     ticpp::Document doc("/home/yucong/.ros/param.xml");
      // actually load the information
     doc.LoadFile();
      //the element to start
     ticpp::Element *child= doc.FirstChildElement("quadrotor_param")->FirstChildElement();
     //an iterator
     ticpp::Iterator<ticpp::Element> iter(child);
     for( iter=iter; iter!=iter.end(); ++iter )
     {
         std::string strName;
         iter->GetValue(&strName);
	 //
	 if( strName== "lambda" )
	   lambda= atof(iter->GetText().c_str() );
         if( strName== "v_obs" )
	   ob_speed= atof(iter->GetText().c_str() );
     }//for child ends

   }
   catch(ticpp::Exception& error)
   {
     cerr << "Error: " << error.m_details << endl;
     return 2;                 // signal error
   }

  //set obstacles
  double Dx= goal_nd.state.x- root_nd.state.x;
  double Dy= goal_nd.state.y- root_nd.state.y;
  double Dz= goal_nd.state.z- root_nd.state.z;
  double DM= sqrt(Dx*Dx+Dy*Dy+Dz*Dz);
  double Dgamma= asin(Dz/DM);
  double Dtheta= atan2(Dy,Dx);

  double x1= root_nd.state.x*(1.-lambda)+goal_nd.state.x*lambda;
  double y1= root_nd.state.y*(1.-lambda)+goal_nd.state.y*lambda;
  double hd1= Dtheta+M_PI;
  double z1= root_nd.state.z*(1.-lambda)+goal_nd.state.z*lambda;
  double v_vert= ob_speed*tan(-Dgamma);
  double t= t_offset;
  double r= 1. ;
  double del_r = r;
  
  ofstream myfile1("obstacles.txt");
  myfile1<<x1<<" "<<y1<<" "<<hd1<<" "<<ob_speed<<" "<<z1<<" "<<v_vert<<" "<<t<<" "<<r<<endl;

  obstacle3D obs1(x1,y1,hd1,ob_speed,z1,v_vert,t,r,del_r);
  vector<obstacle3D> obs_vec(1,obs1);
  /*
  double x2=x1;
  double y2=y1+0;
  double z2=z1-0;
  double hd2= M_PI;
  ob_speed= 0.;
  v_vert= 0.;
  myfile1<<x2<<" "<<y2<<" "<<hd2<<" "<<ob_speed<<" "<<z2<<" "<<v_vert<<" "<<t<<" "<<r<<endl;
   
  obstacle3D obs2(x2,y2,hd2,ob_speed,z2,v_vert,t,r,del_r);
  obs_vec.push_back(obs2);
  double x3=x1;
  double y3=y1-6;
  double z3=z1+0;
  double hd3= M_PI;
  ob_speed= 0.;
  v_vert= 0.;
  obstacle3D obs3(x3,y3,hd3,ob_speed,z3,v_vert,t,r,del_r);
  obs_vec.push_back(obs3);
  myfile1<<x3<<" "<<y3<<" "<<hd3<<" "<<ob_speed<<" "<<z3<<" "<<v_vert<<" "<<t<<" "<<r<<endl;
  
  double x4=x1;
  double y4=y1+6;
  double z4=z1-0;
  double hd4= M_PI;
  ob_speed= 0.;
  v_vert= 0.;
  obstacle3D obs4(x4,y4,hd4,ob_speed,z4,v_vert,t,r,del_r);
  myfile1<<x4<<" "<<y4<<" "<<hd4<<" "<<ob_speed<<" "<<z4<<" "<<v_vert<<" "<<t<<" "<<r<<endl;
  obs_vec.push_back(obs4);
  */
  quad.SetObs(obs_vec); 
  return 0;
}//SetNormalObs ends

int ReplanNode::working()
{//the main working trunk  
  //set obstacles
  ObsLog ob_log;
  if(which_obs==0) 
    SetNormalObs();
  else if(which_obs==1){
    //ObsLog ob_log;
    ob_log.read_list("/home/yucong/.ros/record_4.txt");
  }
  else{
    std::runtime_error("wrong which_obs option");
  }
  //else which_obs ends
  
  //overhead
  //for record
  ofstream myfile("virtual_replan_rec.txt");
  QuadState st_pre;//current quad state and previous quad state
  //messages
  yucong_rrt_avoid::DubinPath_msg path_msg;
  yucong_rrt_avoid::IfNewPath_msg if_new_msg;
  
  int case_idx =PATH_CHECK;
  bool if_path_good= false;
  //bool if_new_path= false;
  if_state= false;
  QuadState st_root;
  
  //publishers and subscribers
  //publishers
  ros::Publisher pub_path=nh.advertise<yucong_rrt_avoid::DubinPath_msg>("path",100);
  ros::Publisher pub_if_new=nh.advertise<yucong_rrt_avoid::IfNewPath_msg>("if_new_path",100);
  //subscribers
  ros::Subscriber sub_receive=nh.subscribe("path_rec",100,&ReplanNode::receiveCb,this);
  ros::Subscriber sub_if_new_rec=nh.subscribe("if_new_rec",100,&ReplanNode::recNewCb,this);
  ros::Subscriber sub_reach=nh.subscribe("quad_reach",100,&ReplanNode::reachCb,this);
  ros::Subscriber sub_state =nh.subscribe("quad_state",100,&ReplanNode::stateCb,this);  
  
  //ros sleep for msgs to be stable
  ros::Duration(t_offset).sleep();
  quad.SetStartTime( ros::Time::now() );
  //set obstacle
  quadNode root_node= quad.GetRoot();
  double t_limit= quad.GetTimeLimit();

  int ob_idx=0;
  if(which_obs == 1)
  {
    vector<obstacle3D> obs_vec;
    obstacle3D obs;
    //std::cout<<"obs time now: "<<ros::Time::now().toSec() <<std::endl;
    ob_log.t_obstacle( root_node.state.t,obs,ob_idx );
    obs_vec.clear();
    obs_vec.push_back(obs);
    quad.SetObs(obs_vec);
  }
  //tree expand for the first round
  bool if_first= true;
  quad.SetInRos();
  quad.TreeExpand();
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
	   if_new_msg.if_new_path= true;
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
	 quad.ClearToDefault();
	 quad.TreeClear();
         if(which_obs==1)
	 {
	   //reset obstacles
	   vector<obstacle3D> obs_vec;
           obstacle3D obs;
	   ob_log.t_obstacle( st_root.t,obs,ob_idx );
	   obs_vec.clear();
	   obs_vec.push_back(obs);
	   quad.SetObs(obs_vec);
	 }
	 //reset tempary goal
	 //reset tree root
	 root_node= quadNode(st_root);
	 //cout<<"st_root: "<<st_root.x<<","<<st_root.y<<","<<st_root.z<<","\
	     <<st_root.theta<<","<<st_root.v<<","<<st_root.vz<<","\
	     <<st_root.t<<endl;
	 //cout<<"goal: "<<goal_node.state.x<<","<<goal_node.state.y<<","<<goal_node.state.z<<endl;
	 quad.TreeSetRoot(root_node);
	 //quad.TreeSetGoal(goal_node);
	 //reset sample parameters because they are influenced by goal and root
	 quad.SetSamples3dParas();
	 //tree expand within time limit
	 quad.TreeExpand();
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
  }//while ends
  myfile.close();
  return 0;
}//working() ends
