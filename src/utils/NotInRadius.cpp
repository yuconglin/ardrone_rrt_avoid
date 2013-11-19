#include "armadillo"
namespace utils{
  
  bool NotInRadius(double x_h,double y_h,double the_h,double x_v,double y_v,double rho)
  {  //translate to the origin
     double x1 = x_v- x_h;
     double x2 = y_v- y_h;
     
     arma::vec::fixed<2> X;
     X << x1 << x2;
     arma::mat::fixed<2,2> P;
     //double the = host.theta;
     P << cos(the_h) << sin(the_h) << arma::endr
       << -sin(the_h) << cos(the_h) << arma::endr;
     //rotated to 0 angle
     X = P*X;
     x1 = X(0);
     x2 = X(1);
     double r1 = sqrt( x1*x1 + pow(x2-rho,2) );
     double r2 = sqrt( x2*x2 + pow(x2+rho,2) );
     if( r1>rho && r2>rho ) return true;
     
     return false;

  }//NotInRadius ends
 
}//namespace ends
