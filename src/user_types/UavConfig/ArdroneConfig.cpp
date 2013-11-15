#include "ArdroneConfig.h"
#include "ticpp.h"

namespace user_types{

   int ArdroneConfig::ParamfromXML(const char* pFilename)
   {
      try
      {
        ticpp::Document doc(pFilename);
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
	    if( strName== "velocity_xy" )
	       this->v= atof(iter->GetText().c_str() );
            if( strName== "velocity_z" )
	       this->vz= atof(iter->GetText().c_str() );
            if( strName== "dt")
	       this->dt= atof(iter->GetText().c_str() );
            if( strName== "yaw_rate")
 	       this->yaw_rate= atof(iter->GetText().c_str() );
        }//for child ends
	speed= sqrt(v*v+vz*vz);

      }
      catch(ticpp::Exception& error)
      {
        cerr << "Error: " << error.m_details << endl;
        return 2;                 // signal error
      }
      return 0; 
   }//ParamfromXML ends

   ArdroneConfig::MaxAscend()
   {
      double vz_max= vz;
      return atan2(vz_max,v);
   }//MaxAscend() ends

   ArdroneConfig::Ascend()
   {
      return atan2(vz,v);
   }

};
