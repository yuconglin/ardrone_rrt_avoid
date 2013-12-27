#include "Sampler.hpp"
#include "UavState/GeneralState.h"

namespace user_types{
   Sampler::Sampler():x0(0),y0(0),z0(0),ga0(0),sigma_ga(0),sample_method(0){ }
   
   void Sampler::SetSampleMethod(int _method)
   {
     if( _method!=0&& _method!=1)
     {
       try {
        throw std::runtime_error ("method not 0 or 1!");
       }//try end
       catch (std::runtime_error &e) {
        std::cout << "Caught a runtime_error exception: "
                  << e.what () << '\n';
	//default to 1
	sample_method= 1;
       }//catch ends   
     }
     sample_method = _method;
   }//SetSampleMethod ends
}
