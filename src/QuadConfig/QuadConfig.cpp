#include "ticpp.h"
#include "QuadConfig.hpp"
#include "iostream"

using namespace std;
QuadConfig::QuadConfig(){
   //for vx, vy
    s_vxy_P=0.;
    s_vxy_I=0.;
    s_vxy_D=0.;
    s_vxy_Limit=0.;
   //for vz
    s_vz_P=0.;
    s_vz_I=0.;
    s_vz_D=0.;
    s_vz_Limit=0.;
   //for yaw
    s_yaw_P=0.;
    s_yaw_I=0.;
    s_yaw_D=0.;
    s_yaw_Limit=0.;

    v= 0.;
    yaw_rate= 0.;
    vz= 0.;
    dt= 0.;
}

int QuadConfig::s_ReadFromXML(const char* pFilename)
{//using ticpp
   try
   {
      ticpp::Document doc(pFilename);
      // actually load the information
      doc.LoadFile();
      //the element to start
      ticpp::Element *child = doc.FirstChildElement("robot")->FirstChildElement("xacro:macro")->FirstChildElement("gazebo")->FirstChildElement("controller:hector_gazebo_quadrotor_simple_controller")->FirstChildElement();
      //an iterator
      ticpp::Iterator<ticpp::Element> iter(child);

      for( iter=iter; iter!=iter.end(); ++iter )
      {
         std::string strName;
         iter->GetValue(&strName);

	 if( strName== "yawProportionalGain" )
         {
	   s_yaw_P= atof(iter->GetText().c_str() );
	   //cout<< "yaw_P= "<< s_yaw_P << endl;
         }
         if( strName== "yawDifferentialGain" )
	   s_yaw_D= atof(iter->GetText().c_str() );
         if( strName== "yawLimit")
	   s_yaw_Limit= atof(iter->GetText().c_str() );
         if( strName== "velocityXYProportionalGain")
 	   s_vxy_P= atof(iter->GetText().c_str() );
         if( strName=="velocityXYDifferentialGain")
	   s_vxy_D= atof(iter->GetText().c_str() );
         if( strName== "velocityXYLimit")
 	   s_vxy_Limit= atof(iter->GetText().c_str() );
         if( strName== "velocityZProportionalGain")
	   s_vz_P= atof(iter->GetText().c_str() );
         if( strName== "velocityZDifferentialGain")
	   s_vz_D= atof(iter->GetText().c_str() );
         if( strName== "velocityZLimit")
	   s_vz_Limit= atof(iter->GetText().c_str() );
      }//for child ends
      cout<<"vxy_D= "<< s_vxy_D << endl;
   }
   catch(ticpp::Exception& error)
   {
      cerr << "Error: " << error.m_details << endl;
      return 2;                 // signal error
   }
   return 0;
}//s_ReadFromXML ends
