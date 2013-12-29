#pragma once
#include <iostream>
#include <cmath>
#include <stdexcept>

namespace user_types{
  
     struct obs3D {
            double x;
	    double y;
	    double z;
	    double r;
	    obs3D( ):x(0),y(0),z(0),r(1)
	    {
	    }
	    obs3D(double _x,double _y,double _z,double _r):x(_x),y(_y),z(_z),r(_r)
	    {
	    }
     };

     struct obstacle3D{
            double x1,x2,x3;
	    //double heading;
	    //double speed;
	    //double v_vert;
	    double v1,v2,v3;
	    double t;
	    double r;
	    double delta_r;
	    //constructor
	    //obstacle3D():x1(0),x2(0),x3(0),heading(0),speed(0),v_vert(0),t(0),r(0),delta_r(0){ };
	    //obstacle3D(double _x1, double _x2, double _heading, double _speed, double _x3, double _v_vert, time_t _t, double _r, double _delta_r):
            //         x1(_x1),x2(_x2),x3(_x3),heading(_heading),speed(_speed),v_vert(_v_vert),t(_t),r(_r),delta_r(_delta_r) { };
            obstacle3D():x1(0),x2(0),x3(0),v1(0),v2(0),v3(0),t(0),r(0),delta_r(0){ };
	    obstacle3D(double _x1,double _x2,double _x3,double _v1,double _v2,double _v3,double _t,double _r,double _delta_r):
	       x1(_x1),x2(_x2),x3(_x3),v1(_v1),v2(_v2),v3(_v3),t(_t),r(_r),delta_r(_delta_r){ };

	    obs3D Estimate(double t1) const
	    {
               if(t1<t)
	       {
		 std::cout<<"neg time "<<t1<<" "<< t<<std::endl;
		 try {
                   throw std::runtime_error ("negtive time");
                 }
                 catch (std::runtime_error &e){
                   std::cout << "Caught a runtime_error exception: "
                   << e.what () << '\n';
                 }

	       }
	       //double x=x1+speed*cos(heading)*(t1-t);
               //double y=x2+speed*sin(heading)*(t1-t);
	       //double z=x3+v_vert*(t1-t);
	       double x= x1+v1*(t1-t);
	       double y= x2+v2*(t1-t);
	       double z= x3+v3*(t1-t);
	       return obs3D(x,y,z,r);
	    }

    };//struct ends

};//namespaces ends
