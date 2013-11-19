#pragma once
//a collection of parameters used in dubins check
namespace user_types{
  
  struct checkParas{
       double end_r;
       double ds_check;
       double ds_insert;
       
       checkParas():end_r(0.),ds_check(0.),ds_insert(0.){};

       checkParas(double _end_r,double _ds_check,double _ds_insert):end_r(_end_r),ds_check(_ds_check),ds_insert(_ds_insert){};
  };//checkParas ends

};//namespace ends
