
struct QuadConfig{
   //for vx, vy
   double s_vxy_P;
   double s_vxy_I;
   double s_vxy_D;
   double s_vxy_Limit;
   //for vz
   double s_vz_P;
   double s_vz_I;
   double s_vz_D;
   double s_vz_Limit;
   //for yaw
   double s_yaw_P;
   double s_yaw_I;
   double s_yaw_D;
   double s_yaw_Limit;

   double v;
   double yaw_rate;
   double vz;
   double dt;
   
   QuadConfig();
   //~QuadConfig();
   int s_ReadFromXML(const char* pFilename="/opt/ros/fuerte/stacks/hector_quadrotor/hector_quadrotor_gazebo_plugins/urdf/quadrotor_simple_controller.urdf.xacro");
};
