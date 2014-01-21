#include "ArdroneConfig.h"
#include "ticpp.h"
#include <cmath>
#include "armadillo"

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
	//speed= v;
	rho= v/yaw_rate;
        //rho= speed/yaw_rate;
	//end_r= max(speed*dt,0.15);
      }
      catch(ticpp::Exception& error)
      {
	std::cerr << "Error: " << error.m_details << std::endl;
        return 2;                 // signal error
      }
      return 0; 
   }//ParamfromXML ends

   double ArdroneConfig::MaxAscend()
   {
      double vz_max= vz;
      return atan2(vz_max,v);
   }//MaxAscend() ends

   double ArdroneConfig::Ascend()
   {
      return atan2(vz,v);
   }

   void ArdroneConfig::NormalizeU(arma::vec::fixed<3>& u)
   {
      double uxy_mag= sqrt( u(0)*u(0)+u(1)*u(1) );
      double uz_mag= fabs(u(2));
   
      if(uxy_mag!=0 && uz_mag!=0)
      {
        double cons= std::min(this->v/uxy_mag,this->vz/uz_mag);
        u<< u(0)*cons<< u(1)*cons << u(2)*cons;  
      }
      else if(uxy_mag==0 && uz_mag!=0)
      u<<0.<<0.<<vz/uz_mag;
      else if(uxy_mag!=0 && uz_mag==0)
      {
        double cons= this->v/uxy_mag;
        u<< u(0)*cons<< u(1)*cons<< 0;
      }
      else 
        u<<0.<<0.<<0.;
      //std::cout<<"u: "<< u(0)<<" "<<u(1)<<" "<<u(2)<< std::endl;
      //std::cout<<"u: "<< sqrt(u(0)*u(0)+u(1)*u(1) ) << std::endl;
   }//NormalizeU ends

};
