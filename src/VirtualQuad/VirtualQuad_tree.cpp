#include "VirtualQuad.hpp"
#include "quadDubins3D.h"
#include <boost/random.hpp>
#include "funcs.hpp"
#include <algorithm>
#include <iostream>
#include <ctime>
#include <cmath>
#include <armadillo>

#define SAMPLE_METHOD 1  //0:arbitrary 3D sampling, 1:sampling in a plane
#define IN_ROS 1
#define SAMPLE_NEW 1

bool NotInRadiusXY(const QuadState& host, const QuadState& visit,double rho);

void VirtualQuad::TreeSetRoot(const quadNode& root_nd)
{
   TREEIter root_it = main_tree.insert( main_tree.begin(), root_nd );
   this->root_node = *root_it;
   //push root iter to the vector
   tree_vector.push_back(root_it);
}//TreeSetRoot ends

void VirtualQuad::TreeSetGoal(const quadNode& goal_nd)
{
   this->goal_node= goal_nd;
   if_goal_set= true;
   double D= sqrt( pow(goal_node.state.x-root_node.state.x,2)\
		  + pow(goal_node.state.y-root_node.state.y,2)\
                  + pow(goal_node.state.z-root_node.state.z,2)\
		  );
   ds_check= max(speed*config.dt, D/100);
   ds_insert= 5*ds_check;
}//TreeSetGoal ends

void VirtualQuad::SetSamples3dParas()
{
  assert(if_goal_set== true);
  double Dx= goal_node.state.x-root_node.state.x;
  double Dy= goal_node.state.y-root_node.state.y;
  double Dz= goal_node.state.z-root_node.state.z;
  double theta0= atan2(Dy,Dx);
#if SAMPLE_METHOD ==0
  double r0= sqrt(Dx*Dx+Dy*Dy+Dz*Dz);
  double gamma0= asin(Dz/r0);
#elif SAMPLE_METHOD == 1
  double r0= sqrt(Dx*Dx+Dy*Dy);
  double gamma0= atan2(Dz,r0);
#endif
  
  my_sampleparas.x0 = root_node.state.x;
  my_sampleparas.y0 = root_node.state.y;
  my_sampleparas.z0 = root_node.state.z;
  my_sampleparas.r0 = r0;
  
  my_sampleparas.sigma_r= 0.5*r0;
  my_sampleparas.theta0 = theta0;
  my_sampleparas.sigma_theta= 0.125*M_PI;
  my_sampleparas.ga0 = gamma0;
  my_sampleparas.sigma_ga= atan2(config.vz,config.v)*0.25;
  //my_sampleparas.sigma_ga= 0.125*M_PI;
  if_samplepara_set= true;
	  //for testing
	  //std::cout<<my_sampleparas.x0<<" "<;;<my_sampleparas.y0<<" "<<my_sampleparas.z0<<" "<<my_sampleparas.r0<<" "<<my_sampleparas.sigma_r<<" "<<my_sampleparas.theta0<<" "<<my_sampleparas.sigma_theta<<" "<<my_sampleparas.ga0<<" "<<my_sampleparas.sigma_ga<<std::endl;
}//SetSampleParas ends

void VirtualQuad::SamplingNodesNew()
{//sampling and check combined
  if(!if_samplepara_set) SetSamples3dParas();
  double r= 0;
  while(1)
  {
    boost::mt19937 generator;
    static unsigned int seed = 0;
    generator.seed(static_cast<unsigned int>(std::time(0))+(++seed));
    //sample r
    boost::normal_distribution<> r_distribution(my_sampleparas.r0,my_sampleparas.sigma_r);
    boost::variate_generator<boost::mt19937&,boost::normal_distribution<> > r_nor(generator, r_distribution);  
    r= r_nor();
    //sample theta  
    boost::uniform_real<> the_uniform(my_sampleparas.theta0-M_PI/2.0, my_sampleparas.theta0+M_PI/2.0);
    boost::variate_generator<boost::mt19937&,boost::uniform_real<> > the_uni(generator, the_uniform);
    double theta= the_uni();
    //get x and y
    sample_node.state.x= my_sampleparas.x0+ r*cos(theta);
    sample_node.state.y= my_sampleparas.y0+ r*sin(theta);
    //check
    if( NotInRadiusXY(root_node.state,sample_node.state,rho) )
      break;
  } //while for x,y ends
  //get z
  //sample a gamma
  while(1){
    boost::mt19937 generator;
    static unsigned int seed = 0;
    generator.seed(static_cast<unsigned int>(std::time(0))+(++seed));

    double ga_max= atan2(config.s_vz_Limit,config.v);
    boost::uniform_real<> ga_uniform(-ga_max,ga_max);
    boost::variate_generator< boost::mt19937&,boost::uniform_real<> > ga_uni(generator,ga_uniform);
    double ga= ga_uni();
    double z= my_sampleparas.z0+ r*tan(ga); 
    if( z>H_MIN && z<H_MAX ){
      sample_node.state.z= z;
      break;
    }
  }//while for z ends
  sample_node.state.theta= root_node.state.theta;
}//SamplingNodesNew ends

void VirtualQuad::SamplingNodes()
{
  if(!if_samplepara_set) SetSamples3dParas();
  while(1)
  {
    boost::mt19937 generator;
    static unsigned int seed = 0;
    generator.seed(static_cast<unsigned int>(std::time(0))+(++seed));
    boost::normal_distribution<> r_distribution(my_sampleparas.r0,my_sampleparas.sigma_r);
    boost::variate_generator<boost::mt19937&,boost::normal_distribution<> > r_nor(generator, r_distribution);  
    double r= r_nor();
	    
    boost::uniform_real<> the_uniform(my_sampleparas.theta0-M_PI/2.0, my_sampleparas.theta0+M_PI/2.0);
    boost::variate_generator<boost::mt19937&,boost::uniform_real<> > the_nor(generator, the_uniform);
    double theta= the_nor();
#if SAMPLE_METHOD == 0
    boost::normal_distribution<> ga_distribution(my_sampleparas.ga0, my_sampleparas.sigma_ga);
    boost::variate_generator<boost::mt19937&,boost::normal_distribution<> > ga_nor(generator, ga_distribution); 
    double ga= ga_nor();

    sample_node.state.x= my_sampleparas.x0+ r*cos(theta)*cos(ga);
    sample_node.state.y= my_sampleparas.y0+ r*sin(theta)*cos(ga);
    sample_node.state.z= my_sampleparas.z0+ r*sin(ga);
#elif SAMPLE_METHOD == 1
    sample_node.state.x= my_sampleparas.x0+ r*cos(theta);
    sample_node.state.y= my_sampleparas.y0+ r*sin(theta);
    double d_x= my_sampleparas.x0-sample_node.state.x;
    double d_y= my_sampleparas.y0-sample_node.state.y;
    //get normal vector
    double x_r= root_node.state.x;
    double y_r= root_node.state.y;
    double z_r= root_node.state.z;
    double x_g= goal_node.state.x;
    double y_g= goal_node.state.y;
    double z_g= goal_node.state.z;
	  
    double n_x= -1.*(x_g-x_r)*(z_g-z_r);
    double n_y= -1.*(y_g-y_r)*(z_g-z_r);
    double n_z= pow(x_g-x_r,2)+pow(y_g-y_r,2);
    assert(n_z > 0 );

    sample_node.state.z= my_sampleparas.z0+ 1./n_z*( n_x*d_x+n_y*d_y );
#endif
    if(  NotInRadiusXY(root_node.state,sample_node.state,rho)
      && sample_node.state.z >H_MIN 
      && sample_node.state.z <H_MAX
      )
      break;
  }//while ends
  sample_node.state.theta= root_node.state.theta;
}//sample nodes ends

bool VirtualQuad::CheckSample()//true for good
{
  bool if_inradi=NotInRadiusXY(root_node.state,sample_node.state,rho);
  //cout <<"if_inradi: "<< if_inradi<< endl;
  //check for gamma limit
  DubinsPath path;
  double s_x= sample_node.state.x;
  double s_y= sample_node.state.y;
  double s_z= sample_node.state.z;
  //cout<<"s_x: "<<s_x<<" s_y: "<<s_y<<" s_z: "<<s_z<<endl;
  double q0[]={root_node.state.x,root_node.state.y,root_node.state.theta};
  double q1[]={s_x,s_y,sample_node.state.theta};
  dubins_init(q0,q1,rho,&path);
  
  double length= dubins_path_length(&path);
  double h= abs(s_z-root_node.state.z);
  double gamma_d= atan2(h,length);
  bool if_ga= (gamma_d< atan2(config.s_vz_Limit,config.v) );
  return if_inradi && if_ga;
}//CheckSample() ends

double VirtualQuad::Heuristics( quadNode& node )
{//calculate the heuristics from the node to the sample node
//basically the dubins length plus vertical height but if too steep just 
//assign a large heuristic
  double heuri=0;
  double s_x= sample_node.state.x;
  double s_y= sample_node.state.y;
  double s_z= sample_node.state.z;
  double n_x= node.state.x;
  double n_y= node.state.y;
  double n_z= node.state.z;

  double q0[]= {n_x,n_y,node.state.theta};
  double q1[]= {s_x,s_y,sample_node.state.theta};
  
  DubinsPath path;
  dubins_init(q0, q1, rho, &path);
  double length= dubins_path_length(&path);
  double h= abs(s_z-n_z);
  double gamma_d= atan2(h,length);
  if( gamma_d > atan2(config.s_vz_Limit,config.v) ) 
      heuri= 1e8;
  else
  {
      heuri= sqrt(length*length + h*h);
      boost::mt19937 generator;
      generator.seed(static_cast<unsigned int>(std::time(0)));
      
      boost::uniform_real<double> distribution(0.0,1.0);
      boost::variate_generator<boost::mt19937&,boost::uniform_real<> > p_dis(generator, distribution);

      double p= p_dis();
      if( !if_goal && p<=0.3 || if_goal && p>0.3 )
	heuri= node.cost+ heuri/speed;
  }
  return heuri;
}//Heuristics ends

void VirtualQuad::CalHeuri()
{
  for(TREEIter it=main_tree.begin();it!=main_tree.end();++it)
      it->heuri=Heuristics(*it);	          
}

bool NodeCompFunc(TREEIter it1,TREEIter it2)
{
    return it1->heuri < it1->heuri;
}

void VirtualQuad::SortNodes( )
{
    tree_vector_sort.clear();
    tree_vector_sort = tree_vector;
    std::sort(tree_vector_sort.begin(), tree_vector_sort.end(), NodeCompFunc);
}
        
bool GoalCompFunc(TREEIter it1, TREEIter it2)
{
   return (it1->cost+it1->cost2go) < (it2->cost+it2->cost2go);
}

void VirtualQuad::SetObs(const vector<obstacle3D> obses)
{
   this->obstacles= obses;
}//SetObs ends

bool VirtualQuad::CheckGoalReach(TREEIter it)
//ds is the checking step
{//true if collided
   //cout<< "goal it: "<<it->state.x<<" "<<it->state.y<<" "<<it->state.z<< endl;
   QuadCfg cfg_start(it->state.x,it->state.y,it->state.z,it->state.theta);
   QuadCfg cfg_end(goal_node.state.x,goal_node.state.y,goal_node.state.z,goal_node.state.theta);
   //create a dubins curve connecting the node to the goal
   quadDubins3D dubin_3d(cfg_start,cfg_end,rho);
   //check for rise limit
   if( asin( (goal_node.state.z-it->state.z)/(dubin_3d.GetHeuriLength()) ) >= atan2(config.s_vz_Limit,config.v) )
   {
     //std::cout<<"goal too steep"<<std::endl;
     return false;
   }
   //check for collision
   QuadState st_sample,st_final;
   double delta_t= 1e7;//a very large number
   int colli=DubinsTotalCheck(delta_t,dubin_3d,it->state,cfg_end,st_final,st_sample,obstacles);
   //test goal
   //double dis_coll= pow(cfg_end.x-st_final.x,2)+pow(cfg_end.y-st_final.y,2)+pow(cfg_end.z-st_final.z,2);
   //cout<< sqrt(dis_coll) << endl;

   if(colli== 1)//free of collision
   {
     it->cost2go= this->dubin_actual_length;
     it->goal_reach= true;
     if(!if_goal) if_goal= true;
     return false;
   }//if colli ends
   else
     return true;
}//CheckGoalReach ends

void VirtualQuad::InsertDubinsNode(TREEIter start_it/*which node it inserts to*/)
{
   assert( path_log.size()>0 );
   //path tempary storage bacause GoalReachCheck will rebuild path_log
   vector<QuadState> path_copy= path_log;   
   TREEIter insert_it;
   double step= speed*config.dt; 
   int N_ITER = floor( ds_insert/step );//insert interval
   int _idx_dubin= dubin_collects.size();
   double seg_len = 0.;
   //std::cout<<"insert path_log size: "<< path_copy.size() << std::endl;
   for(int i=1;i!=path_copy.size();++i)
   {
      if(!if_limit_reach)
      {
	//ros::Duration duration= ros::Time::now()-t_start; 
	//sec_count= duration.toSec();
	sec_count= ros::Time::now().toSec()-t_start.toSec();
	//cout<<"sec_count: "<<sec_count<<endl;
	if( sec_count >= t_limit)
	{
	  if_limit_reach= true;
	  cout<<"stop1"<<endl;
	  break;
	}
      }

      if(i% N_ITER==0||i==path_copy.size()-1)
      {
	 if(i== N_ITER) insert_it = start_it;//the first one
	 quadNode cp_node;
	 //std::cout<<"i= "<<i<<std::endl;
	 cp_node.state= path_copy[i];
	 seg_len= i*step;
	 cp_node.cost= start_it->cost+ seg_len;
	 cp_node.idx_dubin= _idx_dubin;
	 cp_node.idx_state= i;
	
	 insert_it=main_tree.append_child(start_it,cp_node);
	 tree_vector.push_back( insert_it );
	 //goal connecting test
	 CheckGoalReach(insert_it);
      }// for i%N_ITER ends
   }//for int i ends

}//InsertDubinsNode ends

void VirtualQuad::TreeExpand()
{
    if(!if_in_ros)
    {
      ros::Time::init();
      t_start = ros::Time::now();//timing start point
    }//if_in_ros ends
    cout<<"tree expand starts"<<endl;
    int sample_count = 0;//effective sample
    int sample_raw= 0;//raw samples
    CheckGoalReach( main_tree.begin());
    
    //check for collision
    QuadState st_sample,st_final;
    double delta_t= 1e7;//a very large number

    //main loop starts here
    while(1)
    {
#if SAMPLE_NEW == 0
      SamplingNodes();
#else
      SamplingNodesNew();
#endif
      ++sample_raw;
      //sample no good, resample!
      if(!CheckSample()) continue;
      if(sample_count==0)
      {
	 QuadCfg cfg_start(root_node.state.x,root_node.state.y,root_node.state.z,root_node.state.theta);
	 QuadCfg cfg_end(sample_node.state.x,sample_node.state.y,sample_node.state.z,sample_node.state.theta);
	 //create a dubins curve connecting the root to the sampled node
	 quadDubins3D dubin_3d(cfg_start,cfg_end,rho);
	 
	 if(asin( (sample_node.state.z-root_node.state.z)/(dubin_3d.GetHeuriLength())) >= atan2(config.s_vz_Limit,config.v) )
	 {
	    //cout<<"too steep"<<endl;
	    continue;
	 }
	 else
	 {
	    int colli=DubinsTotalCheck(delta_t,dubin_3d,root_node.state,cfg_end,st_final,st_sample,obstacles);
	    //cout <<"colli path_log size: "<<path_log.size()<<endl;
	    if(colli!=-1)
	    {
	      ++sample_count;
	      if(path_log.size()>10)
	        InsertDubinsNode( main_tree.begin() ); 
	      path_log.clear();
	      dubin_collects.push_back(dubin_3d);
	      //cout << "sample 1 genertated" <<endl;
	    } //if check ends
	 } //else ends
      }//if(sample_count==0) ends
      else
      {
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
	   QuadState start = (*tree_it).state;
	   QuadCfg cfg_start(start.x,start.y,start.z,start.theta);
	   QuadCfg cfg_end(sample_node.state.x,sample_node.state.y,sample_node.state.z,sample_node.state.theta);
	   //create a dubins curve connecting the node to the sampling node
	   quadDubins3D dubin_3d(cfg_start,cfg_end,rho);

	   if(asin( (sample_node.state.z-start.z)/(dubin_3d.GetHeuriLength()) ) >= atan2(config.s_vz_Limit,config.v) )
	   {
	      //cout<<"too steep"<<endl;
	      continue;
	   }
	   else
	   {
	      int colli=DubinsTotalCheck(delta_t,dubin_3d,start,cfg_end,st_final,st_sample,obstacles);
	      //cout <<"colli path_log size: "<<path_log.size()<<endl;
	      if(colli!=-1)
	      {
		++sample_count;
		if( path_log.size()> 10)
		  InsertDubinsNode( tree_it ); 
		path_log.clear();
		dubin_collects.push_back(dubin_3d);
		//cout << "sample 1 genertated" <<endl;
		break;
	      } //if check ends
	   } //else ends

	 }// for int j ends
      } //sample_count!= 0 ends

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
      
      if(if_limit_reach)//while time limit for tree expanding reached 
      {
	 cout<<"time="<<" "<<sec_count<<" "\
	     <<"nodes="<<" "<<tree_vector.size()<<" "\
	     <<"samples good="<<" "<<sample_count<<" "\
	     <<"smaples="<<" "<< sample_raw <<endl;
	 break;
      }

    }//while ends
    
}//tree expand ends

bool VirtualQuad::PathGen()	
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
      //path_total.clear();
      //SortGoalNodes();
      cout << "goal: "<<goal_node.state.x<<" "<<goal_node.state.y<<" "<<goal_node.state.z<<endl;
      TREEIter it_next= goal_connect_nodes[0];
      //Dubins3D db_3d;
      //db_3d.DubinsInit(it_next->state,goal_node.state,config.turn_radius,config.speed,config.max_prate);
      QuadCfg cfg_start(it_next->state.x,it_next->state.y,it_next->state.z,it_next->state.theta);
      QuadCfg cfg_end(goal_node.state.x,goal_node.state.y,goal_node.state.z,goal_node.state.theta);
      //create a dubins curve connecting the node to the sampling node
      quadDubins3D dubin_3d(cfg_start,cfg_end,rho);

      dubin_collects.push_back(dubin_3d);
      goal_node.idx_dubin= dubin_collects.size()-1;
      it_next =main_tree.append_child(it_next, goal_node);
      //forming the path		   
      while(1)
      {
	QuadState st= it_next->state;
	//cout<<st.x<<" "<<st.y<<" "<<st.z<<" "<<it_next->idx_dubin<<endl;
	path_total.push_back(it_next);
	it_next= main_tree.parent(it_next);
	if( it_next==main_tree.begin() )
	{
	  QuadState st= it_next->state;
	  //cout<<st.x<<" "<<st.y<<" "<<st.z<<" "<<it_next->idx_dubin<<endl;
	  path_total.push_back(it_next);
	  break;
	}
      }
      std::reverse( path_total.begin(),path_total.end() );
      
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
	   
	   double delta_t= 1e8;
           QuadState st_final,st_sample;
	   for(i= i_end; i> i_start+1; --i)
	   {
	     TREEIter it_end= path_total[i];
	     QuadCfg cfg_start(it_start->state.x,it_start->state.y,it_start->state.z,it_start->state.theta);
	     QuadCfg cfg_end(it_end->state.x,it_end->state.y,it_end->state.z,it_end->state.theta);
	     //create a dubins curve connecting the node to the sampling node
	     quadDubins3D dubin_3d(cfg_start,cfg_end,rho);
             
	     int colli=DubinsTotalCheck(delta_t,dubin_3d,it_start->state,cfg_end,st_final,st_sample,obstacles);
             if(colli==1)
	     {
               i_start= i;
	       path_prune.push_back(path_total[i_start]);
	       break;
	     }//if colli!= -1 ends

	   }//for int i ends
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
}

bool VirtualQuad::PathCheck(QuadState& st_init,TREEIter& it_block,vector<QuadState>& traj_rec)
{
  cout<<"*************it is path check***************"<<endl;
  t_start= ros::Time::now();
  assert( path_total.size()>1 );
  traj_rec.clear();
  traj_rec.push_back( st_init );
  //to see which wp it starts from
  double dwp[path_total.size()];//wp 0,1,2,3...path_total.size()-1
  double len_wp[path_total.size()-1];//k points, k-1 segments
  //curve 0,1,2...path_total.size()-2
  int idx= -1;
  double dis_temp= 1e8;
  for(int i=0;i< path_total.size();++i)
  {
    QuadState st= path_total[i]->state;
    double dis=pow(st.x-st_init.x,2)+pow(st.y-st_init.y,2)+pow(st.z-st_init.z,2);
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
	len_wp[i]= path_total[i+1]->idx_state*speed*config.dt;
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
  
  //assert( idx_sec< path_total.size()-1 );
  cout<<"idx: "<<idx<<" idx_sec: "<< idx_sec <<endl;
  QuadState st_cu= st_init,st_next,st_sample;
  //double t_left= t_limit, t_left_next;
  int colli=1;
  double delta_t= 1e8;

  for(int i=idx_sec; i!= path_total.size()-1; ++i)
  {
     //if(i==idx+3) break;
     TREEIter it_wp= path_total[i+1];
     cout<<"xxxxxxxxxxx,it_wp idx_dubin: "<<it_wp->idx_dubin<<endl;
     quadDubins3D dubin_3d= dubin_collects[it_wp->idx_dubin];
     //cout<<"dubin start "<<dubin_3d.cfg_start.x<<" "<<dubin_3d.cfg_start.y<<" "<<dubin_3d.cfg_start.z<<endl;
     //cout<<"dubin end "<<dubin_3d.cfg_end.x<<" "<<dubin_3d.cfg_end.y<<" "<<dubin_3d.cfg_end.z<<endl;
     //if_colli= db_3d.PropTotalCheck(st_cu,it_wp->state,st_next,obstacles);
     QuadCfg cfg_end(it_wp->state.x,it_wp->state.y,it_wp->state.z,it_wp->state.theta );
     colli=DubinsTotalCheck(delta_t,dubin_3d,st_cu,cfg_end,st_next,st_sample,obstacles);
     //cout<<"check colli: "<<colli<<endl;
     cout<<"idx "<<i<<" "<<st_next.x<<" "<<st_next.y<<" "<<st_next.z<<" "<<st_next.t<<endl;
     cout<<"ideal "<<i<<" "<<it_wp->state.x<<" "<<it_wp->state.y<<" "<<it_wp->state.z<<endl;

     //vector<QuadState>* it_exe= db_3d.GetExe();
     if(colli!=1) 
     {
       if(i== path_total.size()-2)
	  it_block= path_total[i-1];
       else
       {
	  it_block= it_wp;
          cout<<"it_block "<<i<<" "<<it_wp->state.x<<" "<<it_wp->state.y<<" "<<it_wp->state.z<<endl;
       }
       break;
     }
     else
     {
       st_cu= st_next;
       traj_rec.insert(traj_rec.end(),path_log.begin(),path_log.end() );
     }
  }//for int i end

  ros::Duration du= ros::Time::now()- t_start;
  cout<<"colli: "<<colli<<endl;
  cout<< "check time total:+++++++++++++++++++++++ "<< du.toSec() << endl;
  return (colli!=1);

}//end PathCheck

bool VirtualQuad::PathCheckRepeat()
{//if collided, re-select path. True if path is good
  bool if_colli= false,if_path= false;
  TREEIter it_block;
  //assert( !(st_current.x==0&&st_current.y==0&&st_current.z==0) );

  while(1)
  {
    //traj_rec.clear();
    if(!PathGen() ) 
    {//no path to the goal
      cout<<"no path, stop or local avoidance"<<endl;
      break;
    }
    if_colli= PathCheck(st_current,it_block,traj_rec);
    if(!if_colli) 
    {
      if_path= true;
      break;
    }
    else
    {
      //remove child tree and regenerate path
      main_tree.erase(it_block); 	      
    }//if-else ends
    cout<<"xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx"<<endl;
  }//while ends
  return if_path;
}//PathCheckRepeat() ends

void VirtualQuad::TimeStateEstimate(double Dt, QuadState& st)
{//to estimate the state from the initial state after time interval Dt and assign it to st
  assert(!traj_rec.empty() );
  QuadState st_start= traj_rec.front();
  if(Dt> config.dt*traj_rec.size() )
  {
     QuadState st_end= traj_rec.back();
     st.x= st_end.x+st_end.v*cos(st_end.theta)*(Dt-st_end.t+st_start.t);
     st.y= st_end.y+st_end.v*sin(st_end.theta)*(Dt-st_end.t+st_start.t);
     st.z= st_end.z+st_end.vz*(Dt-st_end.t+st_start.t);
     st.theta= st_end.theta;
     st.v= st_end.v;
     st.vz= st_end.vz;
     st.t= st_start.t+ Dt;
  }
  else
  {
     QuadState st_close= traj_rec[floor(Dt/config.dt)];  
     st.x= st_close.x+st_close.v*cos(st_close.theta)*(Dt-st_close.t+st_start.t);
     st.y= st_close.y+st_close.v*sin(st_close.theta)*(Dt-st_close.t+st_start.t);
     st.z= st_close.z+st_close.vz*(Dt-st_close.t+st_start.t);
     st.theta= st_close.theta;
     st.v= st_close.v;
     st.vz= st_close.vz;
     st.t= st_start.t+ Dt;
  }//if else ends

}//TimeStateEstimate ends

void VirtualQuad::PathToMsg(yucong_rrt_avoid::DubinPath_msg& path_msg)
{//path to msg
  if(!path_msg.dubin_path.empty()) path_msg.dubin_path.clear();
  //if path contains only one nodes, the quad will just stop and hover
  //int8 type, Quad_msg start,Quad_msg pt_i1,Quad_msg pt_i2,Quad_msg end
  //no path
  if( path_total.size()==0 ) return;
  //there is path
  for(int i=1; i!=path_total.size(); ++i)
  {
     TREEIter it= path_total[i];
     quadDubins3D db_3d= dubin_collects[it->idx_dubin]; 
     yucong_rrt_avoid::DubinSeg_msg db_msg;//actually a dubin_seg message
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
	db_msg.stop_pt.x= it->state.x;
	db_msg.stop_pt.y= it->state.y;
	db_msg.stop_pt.z= it->state.z;
     //insert into the path
     path_msg.dubin_path.push_back(db_msg);
     
  }//for i ends
}//PathToMsg ends

void VirtualQuad::ClearToDefault()
{//clear vectors
  path_log.clear();
  tree_vector.clear();
  tree_vector_sort.clear();
  goal_connect_nodes.clear();
  dubin_collects.clear();
  traj_rec.clear();
  path_total.clear();
  //flags to default
  sec_count= 0;
  if_limit_reach= false;
  if_goal= false;
  //if_goal_set= false;
  if_samplepara_set= false;
}//ClearToDefault() ends

void VirtualQuad::TreeClear()
{//clear the whole tree
  main_tree.erase(main_tree.begin() );
}

//Set randomly moving obstacles
void VirtualQuad::SetObsRanMove(int sta_num, int dy_num)
{
  double x_min = min(root_node.state.x, goal_node.state.x);
  double x_max = max(root_node.state.x, goal_node.state.x);
  double y_min = min(root_node.state.y, goal_node.state.y);
  double y_max = max(root_node.state.y, goal_node.state.y);
  double z_min = min(root_node.state.z, goal_node.state.z);
  double z_max = max(root_node.state.z, goal_node.state.z);

  if( abs(x_min-x_max)<10 )
  {
     x_min = x_min - 40;
     x_max = x_max + 40;
  }
  double x_low,x_up,y_low,y_up;
  if( goal_node.state.x > root_node.state.x)
  {
    x_low= (x_min+x_max)/2.;
    x_up= x_max;
  }
  else
  {
    x_low= x_min;
    x_up= (x_min+x_max)/2.;
  }
	  
  if( abs(y_min-y_max)<10 )
  {
     y_min = y_min - 40;
     y_max = y_max + 40;
  }
  if( goal_node.state.y > root_node.state.y )
  {
    y_low= (y_min+y_max)/2.;
    y_up= y_max;
  }
  else
  {
    y_low= y_min;
    y_up= (y_min+y_max)/2.;
  }
 
  boost::mt19937 rng; 
  rng.seed(static_cast<unsigned int>(std::time(0)));
  
  boost::uniform_real<> x_distribution(x_low,x_up);
  boost::uniform_real<> y_distribution(y_low,y_up);
  boost::uniform_real<> z_distribution(z_min,z_max);//for z
  boost::normal_distribution<> v_distribution(speed,speed*0.1); 
  boost::uniform_real<> lam_distribution(0,1);

  boost::variate_generator<boost::mt19937&,boost::uniform_real<> > x_dis(rng, x_distribution);
  boost::variate_generator<boost::mt19937&,boost::uniform_real<> > y_dis(rng, y_distribution);
  boost::variate_generator<boost::mt19937&,boost::uniform_real<> > z_dis(rng, z_distribution);
  boost::variate_generator<boost::mt19937&,boost::normal_distribution<> > v_dis(rng, v_distribution);
  boost::variate_generator<boost::mt19937&,boost::uniform_real<> > lam_dis(rng, lam_distribution);

  time_t tt= root_node.state.t;
  double r= this->speed*1.0;
  double del_r= 0.2*r;

  ofstream myfile("obstacles.txt");
  for(int i=0;i!=sta_num; ++i)
  {
     double x=x_dis();
     double y=y_dis();
     double z=z_dis();
     obstacles.push_back(obstacle3D(x,y,0,0,z,0,tt,r,del_r) );
     myfile<<x<<" "<<y<<" "<<0<<" "<<0<<" "<<z<<" "<<0<<" "<<tt<<" "<<r<<endl;
  }

  for(int i=0;i!=dy_num; ++i)
  {
     double x=x_dis();
     double y=y_dis();
     double v=v_dis();
     double z=z_dis();
     //get an intermediate point between start and goal
     double lambda= lam_dis();
     double m_x= x_min*lambda + x_max*(1.-lambda);
     double m_y= y_min*lambda + y_max*(1.-lambda);
     double m_z= z_min*lambda + z_max*(1.-lambda);
     double m_the= atan2(m_y-y,m_x-x);
     double m_dis= pow(m_x-x,2)+pow(m_y-y,2)+pow(m_z-z,2);
     m_dis =sqrt(m_dis);
     double m_ga= asin( (m_z-z)/m_dis );
     double v_vert= v*sin(m_ga);
     double v_gnd= sqrt(v*v-v_vert*v_vert);
     myfile<<x<<" "<<y<<" "<<m_the<<" "<<v_gnd<<" "<<z<<" "<<v_vert<<" "<<tt<<" "<<r<<endl;
     obstacles.push_back(obstacle3D(x,y,m_the,v_gnd,z,v_vert,tt,r,del_r));
  }
  myfile.close();
}//set move obstacles ends 

bool NotInRadiusXY(const QuadState& host, const QuadState& visit,double rho)
{
   //translate to the origin
   double x1 = visit.x-host.x;
   double x2 = visit.y-host.y;
   arma::vec::fixed<2> X;
   X << x1 << x2;
   arma::mat::fixed<2,2> P;
   double the = host.theta;
   P << cos(the) << sin(the) << arma::endr
     << -sin(the) << cos(the) << arma::endr;
   //rotated to 0 angle
   X = P*X;
   x1 = X(0);
   x2 = X(1);
   double r1 = sqrt( x1*x1 + pow(x2-rho,2) );
   double r2 = sqrt( x2*x2 + pow(x2+rho,2) );
   if( r1>rho && r2>rho ) return true;
   else return false;

}//NotInRadiusXY ends

