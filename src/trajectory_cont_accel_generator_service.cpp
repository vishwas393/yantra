// Reference:  https://en.wikibooks.org/wiki/Algorithm_Implementation/Linear_Algebra/Tridiagonal_matrix_algorithm

#include "ros/ros.h"
#include "yantra/JointValues.h"
#include "yantra/Position.h"
#include "yantra/PathCoefficient_1d.h"
#include "yantra/PathCoefficient_2d.h"
#include "yantra/InverseKinematics.h"
#include "yantra/TrajectoryGenerator.h"
#include <math.h>
#include <cmath>
#include <iostream>
#include <vector>




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



yantra::PathCoefficient_2d calculate_coefficients(double *t, yantra::JointValues *q, std::vector<double> &pdd, int j)
{
	yantra::PathCoefficient_2d coeff_a;
	for(int m=0; m<5; m++)		// --> 5(Path eq.) = 6(timestamps or points) - 1
	{
		yantra::PathCoefficient_1d tmp;
		double ti = t[m];
		double tii = t[m+1];
		double qi = q[m].j_value[j];
		double qii = q[m+1].j_value[j];
		double qddi = pdd.at(m);
		double qddii = pdd.at(m+1);

		tmp.a.push_back( (qddi - qddii)/(6*(ti - tii)) );
		tmp.a.push_back( (qddii*ti - qddi*tii)/(2*(ti - tii)) );
		tmp.a.push_back( (6*qi - 6*qii - qddi*pow(ti,2) - 2*qddii*pow(ti,2) + 2*qddi*pow(tii,2) + qddii*pow(tii,2) + 2*qddi*ti*tii - 2*qddii*ti*tii)/(6*(ti - tii)) );
		tmp.a.push_back( -(6*qi*tii - 6*qii*ti + 2*qddi*ti*pow(tii,2) - qddi*pow(ti,2)*tii + qddii*ti*pow(tii,2) - 2*qddii*pow(ti,2)*tii)/(6*(ti - tii)) ); 
		
		coeff_a.a_pi.push_back(tmp); 
	}
	return coeff_a;
}



bool callback_fn(yantra::TrajectoryGenerator::Request &req, yantra::TrajectoryGenerator::Response &res)
{

	double dt[] = {req.T[1] - req.T[0], req.T[2] - req.T[1], req.T[3] - req.T[2], req.T[4] - req.T[3], req.T[5] - req.T[4]};
	std::vector<double> a = {0, dt[1] - ((pow(dt[0],2))/dt[1]), dt[2], dt[3]};
	std::vector<double> b = {(2*(dt[0]+dt[1]))+((pow(dt[0],2))*((1/dt[0])+(1/dt[1]))), 2*(dt[1]+dt[2]), 2*(dt[2]+dt[3], (2*(dt[3]+dt[4]))+((pow(dt[4],2))*((1/dt[3])+(1/dt[4]))))};
	std::vector<double> c = {dt[1], dt[2], dt[3] - ((pow(dt[4],2))/dt[3]),0};
	std::vector<double> d = {0, 0, 0, 0};
	std::vector<std::vector<double>> dd;
	
	yantra::PathCoefficient_2d tc;	
	yantra::JointValues p1,p2;
	std::vector<yantra::JointValues> Qn;
	
	p1.j_value = std::vector<double> (5,0);
	p1.j_velocity = std::vector<double> (5,0);
	p1.j_accel = std::vector<double> (5,0);
	p2.j_value = std::vector<double> (5,0);
	p2.j_velocity = std::vector<double> (5,0);
	p2.j_accel = std::vector<double> (5,0);
	Qn = req.Q;


	

	//Calculating coefficients
	for(int i=0; i<5; i++)		//--> 5 = number of joints
	{
		d = d_vector_formation(&Qn[0], &dt[0], i);
		TBDMsolve(a, b, c, d, b.size());		//vector d is also the return value
		d.insert(d.begin(), 0);				//adding initial acceleration
		d.push_back(0);					//adding destination point acceleration
		dd.push_back(d);
	}

	//Adding two virtual points
	for(int i=0; i<5; i++)		//--> % = number of joints
	{
		p1.j_value[i] = (dd.at(i).at(1)*dt[0]*dt[0]/6) + (Qn[0].j_velocity[i]*dt[0]) + (dd.at(i).at(0)*dt[0]*dt[0]/3) + (Qn[0].j_value[i]);
		p2.j_value[i] = (dd.at(i).at(4)*dt[4]*dt[4]/6) + (Qn[3].j_velocity[i]*dt[4]) + (dd.at(i).at(5)*dt[4]*dt[4]/3) + (Qn[3].j_value[i]);

	}

	Qn.insert(Qn.end()-1, p2);
	Qn.insert(Qn.begin()+1, p1);

	for(int i=0; i<5; i++)
	{
		yantra::PathCoefficient_2d coeff = calculate_coefficients(&req.T[0], &Qn[0], dd.at(i), i);
		res.a_qi.push_back(coeff);	
	}
	return true;
}




int main(int argc, char **argv)
{
	
	ros::init(argc, argv, "trajectory_cont_accel_generator_server");
	ros::NodeHandle nh;

	ros::ServiceServer traj_srvc = nh.advertiseService("trajectory_cont_accel_generator_server", callback_fn);
	ROS_INFO("trajectory_generator_server Online!");
	ros::spin();

	return 0;
}

