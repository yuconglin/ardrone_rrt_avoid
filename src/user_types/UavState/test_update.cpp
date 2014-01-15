#include <fstream>
#include "ArdroneState.h"
#include "armadillo"

using namespace std;
//using namespace Ardrone_rrt_avoid;
using namespace user_types;

int main(int argc, char** argv)
{
   ArdroneState quad_state;
   quad_state.z= 1.2;
   double dt= 0.1;
   double t_limit= 3.7;
   //velocity command
   arma::vec::fixed<3> u;
   u<< 1.0<< 0.0 << 0.;

   //log file
   ofstream myfile("../data/update_rec.txt");

   while( quad_state.t<= t_limit)
   {
     quad_state.LogData(myfile);
     quad_state.Update(u,dt);
   }//end while

   //close the log file
   myfile.close();
}//main ends
