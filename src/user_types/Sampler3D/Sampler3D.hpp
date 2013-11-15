namespace user_types {
  class Sampler3D{
    
    public:
      Sampler3D();
      Sampler3D(double _x0,double _y0,double _z0,double _r0,double _sigr,double _th0, double _sigth, double _ga0, double _sigga);
      inline void SetSigmaGa(double _sigma_ga){this->sigma_ga=_sigma_ga;}
      //set params from root and goal
      SetParams(double x_root,double y_root,double z_root,double x_goal,double y_goal,double z_goal);
      //GetSample
      GetSample(double& x_a, double& y_a, double& z_a);
      //set sampling method
      SetSampleMethod(int _method);
    
    private:
      //parameters
      double x0;
      double y0;
      double z0;
      double r0;
      double sigma_r;
      double theta0;
      double sigma_theta;
      double ga0;
      double sigma_ga;
      //sample method
      int sample_method;
      //uav config
      //GeneralConfig* config_pt;
      //space limit
      SpaceLimit* splimit_pt;
  }//Sampler3D ends

};//namespace user_types ends
