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




void TBDMsolve(std::vector<double>& a, std::vector<double>& b,
	   std::vector<double>& c, std::vector<double>& d, int n)
{
	n--; // since we start from x0 (not x1)
	c[0] /= b[0];
	d[0] /= b[0];

	for (int i = 1; i < n; i++) 
	{
		c[i] /= b[i] - a[i]*c[i-1];
		d[i] = (d[i] - a[i]*d[i-1]) / (b[i] - a[i]*c[i-1]);
	}		

	d[n] = (d[n] - a[n]*d[n-1]) / (b[n] - a[n]*c[n-1]);

	for (int i = n; i-- > 0;) 
	{
		d[i] -= c[i]*d[i+1];
	}
}



std::vector<double> d_vector_formation(yantra::JointValues *q, double *t, int j)
{
	std::vector<double> d (4,0);
	
	d.at(0) = 6 * ((q[0].j_value[j]/t[0]) - (((q[0].j_velocity[j]*t[0]) + (q[0].j_accel[j]*t[0]*t[0]/3) + (q[0].j_value[j]))*((1/t[0])+(1/t[1]))) + (q[1].j_value[j]/t[1]) - (q[0].j_accel[j]*t[0]/6)) ; 
	

	d.at(1) = 6 * ((((q[0].j_velocity[j]*t[0]) + (q[0].j_accel[j]*t[0]*t[0]/3) + (q[0].j_value[j]))/t[1]) - (q[1].j_value[j]*((1/t[1])+(1/t[2]))) + (q[2].j_value[j]/t[2])) ;
	

	d.at(2) = 6 * ((q[1].j_value[j]/t[2]) - (q[2].j_value[j]*((1/t[2])+(1/t[3]))) + (((-q[3].j_velocity[j]*t[4]) + (q[3].j_accel[j]*t[4]*t[4]/3) + (q[3].j_value[j]))/t[3])) ;
	

	d.at(3) = 6 * ((q[2].j_value[j]/t[3]) - (((-q[3].j_velocity[j]*t[4]) + (q[3].j_accel[j]*t[4]*t[4]/3) + (q[3].j_value[j]))*((1/t[3])+(1/t[4]))) + (q[3].j_value[j]/t[4]) - (q[3].j_accel[j]*t[4]/6)) ;

	return d;
}


bool callback_fn(yantra::TrajectoryGenerator::Request &req, yantra::TrajectoryGenerator::Response &res)
{

	double dt[] = {req.T[1] - req.T[0], req.T[2] - req.T[1], req.T[3] - req.T[2], req.T[4] - req.T[3], req.T[5] - req.T[4]};
	std::vector<double> a = {0, dt[1] - ((pow(dt[0],2))/dt[1]), dt[2], dt[3]};
	std::vector<double> b = {(2*(dt[0]+dt[1]))+((pow(dt[0],2))*((1/dt[0])+(1/dt[1]))), 2*(dt[1]+dt[2]), 2*(dt[2]+dt[3], (2*(dt[3]+dt[4]))+((pow(dt[4],2))*((1/dt[3])+(1/dt[4]))))};
	std::vector<double> c = {dt[1], dt[2], dt[3] - ((pow(dt[4],2))/dt[3]),0};
	std::vector<double> f = {0, 0, 0, 0};
	yantra::TrajCoefficient tc;


	for(int i=0; i<5; i++)
	{
		std::vector<double> d = d_vector_formation(&req.Q[0], &dt[0], i);
		//TBDM_Thomas_algo(a, b, c, d, f);
		//tc.a = f;
		TBDMsolve(a,b,c,d,4);
		tc.a = d;
		res.coeff.push_back(tc);
	}
	return true;
}




int main(int argc, char **argv)
{
	
	ros::init(argc, argv, "trajectory_generator_server");
	ros::NodeHandle nh;

	ros::ServiceServer traj_srvc = nh.advertiseService("trajectory_generator_server", callback_fn);
	ROS_INFO("trajectory_generator_server Online!");
	ros::spin();

	return 0;
}

