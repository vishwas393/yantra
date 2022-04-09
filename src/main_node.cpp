#include "ros/ros.h"
#include "yantra/Position.h"
#include "yantra/TrajCoefficient.h"
#include "yantra/InverseKinematics.h"
#include "yantra/TrajectoryGenerator.h"
#include <math.h>

int passing_points = 4;


yantra::JointValues make_JointValues(double *q_val, double *q_velocity, double *q_accel)
{
	std::vector<double> tmp (5);

	yantra::JointValues ret;
	
	for(int j=0; j<5; j++) {
		tmp.at(j) = q_val[j]; }
	ret.j_value = tmp;
	
	for(int j=0; j<5; j++) {
		tmp.at(j) = q_velocity[j]; }
	ret.j_velocity = tmp;
	
	for(int j=0; j<5; j++) {
		tmp.at(j) = q_accel[j]; }
	ret.j_accel = tmp;

	return ret;
}




int apply_IK(ros::ServiceClient *cl, double *pos, double *q_init, double *q)
{
	int err_code = -5;
	std::vector<double> tmp (5);
	yantra::InverseKinematics srv;
	
	for(int j=0; j<5; j++) {
		tmp.at(j) = q_init[j]; }
	
	srv.request.Q_init.j_value = tmp;

	
	std::vector<double> tmp1 (3);
	
	for(int j=0; j<3; j++) {
		tmp1.at(j) = pos[j]; }	
	
	srv.request.P.pos = tmp1;

	if(cl->call(srv))
	{
		ROS_INFO("IK service call successfull!");
		for(int j=0; j<5; j++) {
			q[j] = srv.response.Q.j_value[j]; }
		err_code = 0;
	}

	else
	{
		ROS_INFO("IK service call failed!");
		err_code = -1;
	}

	return err_code;
}



int trajectory_generator(ros::ServiceClient *cl, yantra::JointValues *q, double *a)
{
	yantra::TrajectoryGenerator srv;
	//std::vector<double> tmp (5,0);
	//std::vector<double> tmp1 (5,0);
	//std::vector<double> tmp2 (5,0);

	yantra::JointValues tmp_j;

	for(int k=0; k<passing_points; k++) {
		tmp_j = make_JointValues(&(q[k].j_value[0]), &q[k].j_velocity[0], &q[k].j_accel[0]);
		srv.request.Q.push_back(tmp_j);
	}

	//std::vector<double> time = {0, 1.2, 2.34, 5.56, 7.56, 10};
	std::vector<double> time = {0, 0.10, 0.23, 0.50, 0.76, 0.10};
	srv.request.T = time;

	if(cl->call(srv))
	{
		for(int i=0; i<5; i++) {
			for(int j=0; j<4; j++) {
				*(a+(4*i)+j) = srv.response.coeff[i].a[j];
			}
		}
	}
	else
	{
		ROS_INFO("trajectory_generator_server call falied!");
	}

	return true;
}



int main(int argc, char** argv)
{
	ros::init(argc, argv, "main_node");
	ros::NodeHandle node;

	ros::ServiceClient client_IK = node.serviceClient<yantra::InverseKinematics>("inverse_kinematics_server");	//IK = Inverse Kinematics
	ros::ServiceClient client_TG = node.serviceClient<yantra::TrajectoryGenerator>("trajectory_generator_server");	//TG = Trajectory Generator
	
	ros::Duration wait_time_server(15);

	client_IK.waitForExistence(wait_time_server);


	double q_init[] = {M_PI/4, 0, 0, 0, 0};
	double pos[passing_points][3] = {{190, 180, 200} , {210, 220, 200}, {240, 240, 250} , {300, 300, 300}};
	double q_j_value[passing_points][5];
	double q_j_velocity[passing_points][5] = {0};
	double q_j_accel[passing_points][5] = {0};
	double trajcoeffs[passing_points-1][5];
	yantra::JointValues t[passing_points];
	//double a[5][passing_points+1][4];   <-- Correct one
	double a[5][4];

	for (int i=0; i<passing_points; i++)
	{
		int err = -5;
		int attempt = 0;
		while(err != 0 && attempt<3) {
			err = apply_IK(&client_IK, &pos[i][0], &q_init[0], &q_j_value[i][0]);
			attempt += 1;
		}
	}

	//Just Printing the values the Joint Angles for 6 points
	std::cout << "Printing values of joint angles for given points" << std::endl;
	for(int i=0; i<passing_points; i++) {
		for(int j=0; j<5; j++) {
			std::cout << q_j_value[i][j] << " , ";
		}
		std::cout << std::endl;
	}

	
	for(int i=0; i<passing_points; i++)
	{
		t[i] = make_JointValues(&q_j_value[i][0], &q_j_velocity[i][0], &q_j_accel[i][0]);
	}
	int err = trajectory_generator(&client_TG, &t[0], &a[0][0]);

	std::cout << "Printing the values of Trajectory coefficients" << std::endl;
	for(int i=0; i<5; i++) {
		std::cout << "[";
		for(int j=0; j<4; j++) {
			std::cout << a[i][j] << " , ";
		}
		std::cout << "];" << std::endl;
	}
}
