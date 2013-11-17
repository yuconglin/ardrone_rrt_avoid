#include "PointInPoly.h"
#include <vector>

namespace utils{

 bool PointInPoly( std::vector<user_types::point2D> vertex, double x, double y)
 {   
   int nvert= vertex.size();
   if(nvert<3) return true; //we got an infinitely large region
   int i, j;
   bool c = false;
   for(i=0,j=nvert-1; i<nvert; j=i++)
   {
      double i_x= vertex[i].x, i_y= vertex[i].y;
      double j_x= vertex[j].x, j_y= vertex[j].y;
      if ( ((i_y > y) != (j_y> y) ) &&
 	 (x < (j_x-i_x) * (y-i_y) / (j_y-i_y) + i_x) )
        c = !c;
   }
   return c;
 }//PointInPoly ends

};//namespace ends
