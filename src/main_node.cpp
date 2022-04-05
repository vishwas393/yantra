#include "ros/ros.h"
#include "yantra/JointValues.h"
#include "yantra/Position.h"
#include "yantra/TrajCoefficient.h"
#include "yantra/InverseKinematics.h"
#include <math.h>



int apply_IK(ros::ServiceClient *cl, double *pos, double *q_init, double *q)
{
	std::cout << "apply_IK called" << std::endl;	
	int err_code = -5;
	std::vector<double> tmp (5);
	yantra::InverseKinematics srv;
	
	for(int j=0; j<5; j++) {
		tmp.at(j) = q_init[j]; }
	
	srv.request.Q_init.j_values = tmp;

	
	std::vector<double> tmp1 (3);
	
	for(int j=0; j<3; j++) {
		tmp1.at(j) = pos[j]; }	
	
	srv.request.P.pos = tmp1;

	if(cl->call(srv))
	{
		ROS_INFO("IK service call successfull!");
		for(int j=0; j<5; j++) {
			q[j] = srv.response.Q.j_values[j]; }
		err_code = 0;
	}

	else
	{
		ROS_INFO("IK service call failed!");
		err_code = -1;
	}

	return err_code;
}


	double q_init[] = {M_PI/4, 0, 0, 0, 0};
	double pos[6][3] = {{190, 180, 200} , {210, 220, 200}, {241.5, 241.5, 474.38} , {270, 270, 400}, {300, 300, 300}, {350, 350, 350}};
	double q[3][5];


int main(int argc, char** argv)
{
	ROS_INFO("In main");
	ros::init(argc, argv, "main_node");
	ROS_INFO("Step 0");
	ros::NodeHandle node;

	ROS_INFO("Step 1");
	ros::ServiceClient client_IK = node.serviceClient<yantra::InverseKinematics>("inverse_kinematics_server");
	
	ros::Duration wait_time_server(5);

	ROS_INFO("Step 2");
	client_IK.waitForExistence(ros::Duration(-1));
	int passing_points = 6;

	ROS_INFO("Entering apply_IK");
	for (int i=0; i<passing_points; i++)
	{
		int err = -5;
		int attempt = 0;
		while(err != 0 && attempt<3) {
			err = apply_IK(&client_IK, &pos[i][0], &q_init[0], &q[i][0]);
			attempt += 1;
		}
	}


	for(int i=0; i<6; i++) {
		for(int j=0; j<5; j++) {
			std::cout << q[i][j] << " | ";
		}
		std::cout << std::endl;
	}
}
