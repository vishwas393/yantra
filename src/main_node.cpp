#include "ros/ros.h"
#include "yantra/Position.h"
#include "yantra/PathCoefficient_1d.h"
#include "yantra/PathCoefficient_2d.h"
#include "yantra/InverseKinematics.h"
#include "yantra/TrajectoryGenerator.h"
#include "std_msgs/Float64MultiArray.h"
#include <math.h>
#include <vector>

int passing_points = 4;

typedef std::vector<std::vector<double>> array2d;
typedef std::vector<std::vector<std::vector<double>>> array3d;

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



int trajectory_generator(ros::ServiceClient *cl, yantra::JointValues *q, array3d* a)
{
	yantra::TrajectoryGenerator srv;
	array3d tmp2;

	yantra::JointValues tmp_j;

	for(int k=0; k<passing_points; k++) {
		tmp_j = make_JointValues(&(q[k].j_value[0]), &q[k].j_velocity[0], &q[k].j_accel[0]);
		srv.request.Q.push_back(tmp_j);
	}

	//std::vector<double> time = {0, 1.2, 2.34, 5.56, 7.56, 10};
	std::vector<double> time = {0, 0.10, 0.23, 0.50, 0.76, 1.0};
	srv.request.T = time;

	if(cl->call(srv))
	{
		for(int i=0; i<5; i++) {
			std::cout << "Trajectory Coefficient for joint " << i << ":" << std::endl;
			array2d tmp1;
			for(int j=0; j<5; j++) {
				std::vector<double> tmp;
				for(int k=0; k<4; k++) {
					std::cout << srv.response.a_qi[i].a_pi[j].a[k] << " | ";
					tmp.push_back(srv.response.a_qi[i].a_pi[j].a[k]);
				}
				tmp1.push_back(tmp);
				std::cout << std::endl;
			}
			a->push_back(tmp1);
			std::cout<<std::endl;
		}
	}
	else
	{
		ROS_INFO("trajectory_generator_server call falied!");
		return -1;
	}

	return 0;
}



int main(int argc, char** argv)
{
	ros::init(argc, argv, "main_node");
	ros::NodeHandle node;

	ros::ServiceClient client_IK = node.serviceClient<yantra::InverseKinematics>("inverse_kinematics_server");	//IK = Inverse Kinematics
	ros::ServiceClient client_TG = node.serviceClient<yantra::TrajectoryGenerator>("trajectory_generator_server");	//TG = Trajectory Generator
	ros::Publisher pub_jointvalue = node.advertise<std_msgs::Float64MultiArray>("/yantra/yantra_arm_controller/command", 1);
	ros::Duration wait_time_server(15);

	client_IK.waitForExistence(wait_time_server);
	client_TG.waitForExistence(wait_time_server);


	double q_init[] = {M_PI/4, 0, 0, 0, 0};
	double pos[passing_points][3] = {{190, 180, 200} , {210, 220, 200}, {240, 240, 250} , {300, 300, 300}};
	double q_j_value[passing_points][5];
	double q_j_velocity[passing_points][5] = {0};
	double q_j_accel[passing_points][5] = {0};
	double trajcoeffs[passing_points-1][5];
	yantra::JointValues waypoint_joint_space[passing_points];
	array3d coeff_a;


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
		waypoint_joint_space[i] = make_JointValues(&q_j_value[i][0], &q_j_velocity[i][0], &q_j_accel[i][0]);
	}
	
	int err = trajectory_generator(&client_TG, &waypoint_joint_space[0], &coeff_a);
	if(err != 0) {
		std::cout << "No Trajectory coefficient was generated!" << std::endl;
		return 0;
	}


	/* Applying coefficient sending the the position values */
	double t = 0.05;
	std::vector<double> coeff;
	for(int i=0; i<5; i++)
	{
		double one_pt = (coeff_a[i][0][0]*(std::pow(t,3)))+(coeff_a[i][0][1]*(std::pow(t,2)))+(coeff_a[i][0][2]*(t))+(coeff_a[i][0][3]);
		coeff.push_back(one_pt);
	}
	std_msgs::Float64MultiArray pubmsg;
	pubmsg.data = coeff;
	pub_jointvalue.publish(pubmsg);

}
