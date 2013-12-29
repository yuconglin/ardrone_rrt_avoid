#include "ObsUpdaterVirtual.hpp"
#include "ObsLog.h"

namespace user_types{
  ObsUpdaterVirtual::ObsUpdaterVirtual(std::vector<std::string> filenames,double _r,double _del_r)
  {
    if(filenames.size()==0)
    {
      try {
        throw std::runtime_error ("ObsUpdaterVirtual: filenames empty");
      }
      catch (std::runtime_error &e) {
        std::cout << "Caught a runtime_error exception: "
                  << e.what () << '\n';
      } 
      return;
    }

    for(int i=0;i!=filenames.size();++i)
    {
      ObsLog* obslog_pt= new ObsLog(_r,_del_r);
      obslog_pt->read_list( filenames[i].c_str() );
      obslogs.push_back(obslog_pt);
    }//for int i ends

  }//ObsUpdaterVirtual ends

  ObsUpdaterVirtual::~ObsUpdaterVirtual()
  {
    for(int i=0; i!= obslogs.size(); ++i)
      delete obslogs[i];
  }//destructor ends

  void ObsUpdaterVirtual::UpdateObs(std::vector<obstacle3D>& obs3ds,double t)
  {
    obs3ds.clear();

    for(int i=0; i!= obslogs.size(); ++i)
    {
      obstacle3D obs3d;  
      obslogs[i]->t_obstacle(t,obs3d);
      obs3ds.push_back(obs3d);
    }//for ends
  
  }//UpdateObs

}//namespace ends
