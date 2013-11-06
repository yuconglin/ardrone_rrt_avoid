#include "GetRho.h"
#include "ticpp.h"

namespace utils{

int GetRho(double& rho)
{
  double v,yaw_rate;
  try
  {
    const char* pFilename="/home/yucong/.ros/param.xml";
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
	 v= atof(iter->GetText().c_str() );
       if( strName== "yaw_rate")
	 yaw_rate= atof(iter->GetText().c_str() );
    }//for child ends

  }
  catch(ticpp::Exception& error)
  {
    std::cerr << "Error: " << error.m_details << std::endl;
    return 2;                 // signal error
  }
  rho= v/yaw_rate;
  return 0;
}//GetRho ends

}
