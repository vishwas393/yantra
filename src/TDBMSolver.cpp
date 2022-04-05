// Reference: https://www.quantstart.com/articles/Tridiagonal-Matrix-Algorithm-Thomas-Algorithm-in-C/


#include "ros/ros.h"
#include "yantra/JointValues.h"
#include "yantra/Position.h"
#include "yantra/TrajCoefficient.h"
#include "yantra/InverseKinematics.h"
#include "yantra/TrajectoryGenerator.h"
#include <math.h>
#include <cmath>
#include <iostream>
#include <vector>


//The first three parameters a,b and c represent the elements in the tridiagonal bands. Since b represents the diagonal elements it is one element longer than a and c, which represent the off-diagonal bands. The latter two parameters represent the solution vector f and the right-hand column vector d.
void TBDM_Thomas_algo(const std::vector<double>& a,
		      const std::vector<double>& b,
		      const std::vector<double>& c,
		      const std::vector<double>& d,
		      std::vector<double>& f)
{
	size_t N = d.size();

	std::vector<double> c_tmp(N, 0.0);
	std::vector<double> d_tmp(N, 0.0);

	c_tmp[0] = c[0] / b[0];
	d_tmp[0] = d[0] / b[0];

	for (int i=1; i<N; i++)
	{
		double m = 1.0 / (b[i] - a[i] * c_tmp[i-1]);
		c_tmp[i] = c[i] - m;
		d_tmp[i] = (d[i] - a[i] * d_tmp[i-1]) * m; 
	}

	for (int i=N-1; i-- >0; )
	{
		f[i] = d_tmp[i] - c_tmp[i] * d[i+1];
	}

}


bool callback_fn(yantra::TrajectoryGenerator::Request &req, yantra::TrajectoryGenerator::Response &res)
{

}




int main(int argc, char **argv)
{
	
	ros::init(argc, argv, "trajectory_generator_server");
	ros::NodeHandle nh;

	ros::ServiceServer traj_srvc = nh.advertiseService("trajectory_generator_server", callback_fn);
	ROS_INFO("trajectory_generator_server Online!");
	ros::spin()
	size_t N = 4;
	double dt = 10;
	std::vector<double> a = {0, dt, dt};
	std::vector<double> b = {6*dt, 4*dt, 4*dt, 6*dt};
	std::vector<double> c = {dt, dt, 0};
	std::vector<double> d = {21, 45, 50, 36};
	std::vector<double> f = {0, 0, 0, 0};

	TBDM_Thomas_algo(a, b, c, d, f);

	std::cout << "f = (";
  	for (int i=0; i<N; i++)
	{
    		std::cout << f[i];
    		if (i < N-1)
		{
      			std:: cout << ", ";
    		}
  	}
  	std::cout << ")" << std::endl;
	return 0;
}
