#pragma once
//#include <iostream>
//#include <cmath>
namespace user_types{

    struct obstacle2D {
            double x;
	    double y;
	    double r;
	    double delta_r;
	    obstacle2D( ):x(0),y(0),r(0),delta_r(0)
	    {
	    }
	    obstacle2D(double _x,double _y,double _r,double _delta_r):x(_x),y(_y),r(_r),delta_r(_delta_r)
	    {
	    }
    };

};//namespace ends
