namespace strucs {

struct SampleParas3D {
            double x0;
	    double y0;
	    double z0;
	    double r0;
	    double sigma_r;
	    double theta0;
	    double sigma_theta;
	    double ga0;
	    double sigma_ga;

	    SampleParas3D() {
              x0 = 0;
	      y0 = 0.;
	      z0 = 0.;
	      r0 = 0.;
	      sigma_r = 0.;
	      theta0 = 0.;
	      sigma_theta = 0.;
              ga0 = 0.;
	      sigma_ga = 0.;
	    }

	    SampleParas3D(double _x0,double _y0,double _z0,double _r0,double _sigr,double _th0, double _sigth, double _ga0, double _sigga) {
              x0 = _x0;
	      y0 = _y0;
	      z0 = _z0;
	      r0 = _r0;
	      sigma_r = _sigr;
	      theta0 = _th0;
	      sigma_theta = _sigth;
	      ga0 = _ga0;
              sigma_ga = _sigga;
	    }
     };

};

