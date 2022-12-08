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


int calculate_imposed_velocity(std::vector<yantra::JointValues> &_qn, std::vector<double> &_time)
{
	//std::vector<double> joint_value = _qn.j_value;
	//std::vector<double> velocity_value = _qn.j_velocity;
	std::vector<double> velocity (4, 0);
	std::vector<double> sign (3,0);

	//Calculating Vi,k
	for(int i=1; i<_qn.j_value.size(); i++)
	{
		velocity.at(i) = (_qn.j_value.at(i) - _qn.j_value.at(i-1)) / (_time.at(i) - _time.at(i-1)) ;
	}

	//Calculating sgn(Vi,k)
	for(int i=1; i<_qn.j_value.size()-1; i++)
	{
		int sgn1 = (velocity.at(i) > 0) ? 1 : ((velocity.at(i) < 0) ? -1 : 0);
		int sgn2 = (velocity.at(i+1) > 0) ? 1 : ((velocity.at(i+1) < 0) ? -1 : 0);

		_qn.j_velocity.at(i) = (sgn1 == sgn2) ? 0.5*(velocity.at(i) + velocity.at(i+1)) : 0;
	}


	

	_qn.j_velocity.at(1) = (sign(1) == sign(2)
}


bool callback_fn(yantra::TrajectoryGenerator::Request &req, yantra::TrajectoryGenerator::Response &res)
{
	std::vector<yantra::JointValues> Qn;
	std::vector<double> time;
	time = req.T;
	Qn = req.Q;

	int ret = calculate_imposed_velocity(Qn, time);
}




int main(int argc, char **argv)
{
	
	ros::init(argc, argv, "trajectory_imposed_vel_generator_server");
	ros::NodeHandle nh;

	ros::ServiceServer traj_srvc = nh.advertiseService("trajectory_imposed_vel_generator_server", callback_fn);
	ROS_INFO("trajectory_imposed_vel_generator_server Online!");
	ros::spin();

	return 0;
}

