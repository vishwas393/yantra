#include "ros/ros.h"
#include "ros/console.h"
#include "yantra/Position.h"
#include "yantra/PathCoefficient_1d.h"
#include "yantra/PathCoefficient_2d.h"
#include "yantra/InverseKinematics.h"
#include "yantra/TrajectoryGenerator.h"
#include "yantra/PathPointsGui.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
#include "controller_manager_msgs/SwitchController.h"
#include "controller_manager/controller_manager.h"
//#include "yantra/direct_kinematic.h"
#include <cmath>
#include <vector>
#include <cstring>

int passing_points = 4;

typedef std::vector<std::vector<double>> array2d;
typedef std::vector<std::vector<std::vector<double>>> array3d;




class Publish_Timer
{
	public:
		Publish_Timer(ros::NodeHandle& _nh, array3d& _a, std::vector<double> _t)
		{
			coeff_a = _a;
			time = _t;
			time_len = time.size();
			end_time = time.at(time_len - 1) + time_step;
			time_count = time.at(0); 

			pub_joint1value = _nh.advertise<std_msgs::Float64>("/yantra/link_one_vel_controller/command", 1);
			pub_joint2value = _nh.advertise<std_msgs::Float64>("/yantra/link_two_vel_controller/command", 1);
			pub_joint3value = _nh.advertise<std_msgs::Float64>("/yantra/link_three_vel_controller/command", 1);
			pub_joint4value = _nh.advertise<std_msgs::Float64>("/yantra/link_four_vel_controller/command", 1);
			pub_joint5value = _nh.advertise<std_msgs::Float64>("/yantra/link_five_vel_controller/command", 1);

			std::cout << "Timer class instance created!" <<std::endl;
		}

		void callback(const ros::TimerEvent& event)
		{
			ROS_INFO_STREAM("end time is " << end_time << "  and time_count is " << time_count);
			
			if((floorf(time_count*10)/10) == (floorf(end_time*10)/10))				// To comapre exactly one digit after decimal point 
			{
				stop();
			}
			else {
				
				idx_c = std::find(time.begin(), time.end(), static_cast<int>(time_count)) - time.begin();
				if(idx_c != time_len && idx_c != time.size()-1) {
					path_seg = idx_c;
				}
				
				ROS_INFO_STREAM("path_seg is: " << path_seg);
				jvalpub.at(0) = (3*coeff_a[0][path_seg][0]*(std::pow(time_count,2)))+(2*coeff_a[0][path_seg][1]*time_count)+(coeff_a[0][path_seg][2]);
				jvalpub.at(1) = (3*coeff_a[1][path_seg][0]*(std::pow(time_count,2)))+(2*coeff_a[1][path_seg][1]*time_count)+(coeff_a[1][path_seg][2]);
				jvalpub.at(2) = (3*coeff_a[2][path_seg][0]*(std::pow(time_count,2)))+(2*coeff_a[2][path_seg][1]*time_count)+(coeff_a[2][path_seg][2]);
				jvalpub.at(3) = (3*coeff_a[3][path_seg][0]*(std::pow(time_count,2)))+(2*coeff_a[3][path_seg][1]*time_count)+(coeff_a[3][path_seg][2]);
				jvalpub.at(4) = (3*coeff_a[4][path_seg][0]*(std::pow(time_count,2)))+(2*coeff_a[4][path_seg][1]*time_count)+(coeff_a[4][path_seg][2]);

				pub_1msg.data = jvalpub.at(0);
				pub_2msg.data = jvalpub.at(1);
				pub_3msg.data = jvalpub.at(2);
				pub_4msg.data = jvalpub.at(3);
				pub_5msg.data = jvalpub.at(4);
				pub_joint1value.publish(pub_1msg);
				pub_joint2value.publish(pub_2msg);
				pub_joint3value.publish(pub_3msg);
				pub_joint4value.publish(pub_4msg);
				pub_joint5value.publish(pub_5msg);

				ROS_INFO_STREAM("Point number: " << time_count);
				time_count += time_step;
				ROS_INFO_STREAM("time_count becomes" << time_count);
			}
		}	

		void start(ros::NodeHandle& _nh)
		{
			timer = _nh.createTimer(ros::Duration(time_step), &Publish_Timer::callback, this);
			std::cout << "Timer started!" << std::endl;
		}

		void stop()
		{
			ROS_INFO_STREAM("Timer stops now");

			pub_1msg.data = 0;
			pub_2msg.data = 0;
			pub_3msg.data = 0;
			pub_4msg.data = 0;
			pub_5msg.data = 0;
			pub_joint1value.publish(pub_1msg);
			pub_joint2value.publish(pub_2msg);
			pub_joint3value.publish(pub_3msg);
			pub_joint4value.publish(pub_4msg);
			pub_joint5value.publish(pub_5msg);
			
			timer.stop();
		}

	private:
		double time_count = 2;
		double time_step = 0.2;
		double end_time = 0;
		int time_len = 0;
		int path_seg = 0;
		int idx_c = 0;
		ros::Timer timer;
		array3d coeff_a;
		std::vector<double> time;
		std::vector<double> jvalpub = std::vector<double>(5, 0);
		
		std_msgs::Float64 pub_1msg;
		std_msgs::Float64 pub_2msg;
		std_msgs::Float64 pub_3msg;
		std_msgs::Float64 pub_4msg;
		std_msgs::Float64 pub_5msg;

		ros::Publisher pub_joint1value; //= nh->advertise<std_msgs::Float64>("/yantra/link_one_vel_controller/command", 1);
		ros::Publisher pub_joint2value; //= nh->advertise<std_msgs::Float64>("/yantra/link_two_vel_controller/command", 1);
		ros::Publisher pub_joint3value; //= node.advertise<std_msgs::Float64>("/yantra/link_three_vel_controller/command", 1);
		ros::Publisher pub_joint4value; //= node.advertise<std_msgs::Float64>("/yantra/link_four_vel_controller/command", 1);
		ros::Publisher pub_joint5value; //= node.advertise<std_msgs::Float64>("/yantra/link_five_vel_controller/command", 1);
};




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



int trajectory_generator(ros::ServiceClient *cl, yantra::JointValues *q, std::vector<double> time, array3d &a)
{
	yantra::TrajectoryGenerator srv;
	array3d tmp2;

	yantra::JointValues tmp_j;

	for(int k=0; k<passing_points; k++) {
		tmp_j = make_JointValues(&(q[k].j_value[0]), &q[k].j_velocity[0], &q[k].j_accel[0]);
		srv.request.Q.push_back(tmp_j);
	}


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
			a.push_back(tmp1);
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




int call_gui(ros::ServiceClient *cl, std::vector<double> &t, double (*pos)[4][3])
{
	yantra::PathPointsGui	srv;
	srv.request.dummy = 0;

	if(cl->call(srv))
	{
		ROS_INFO("GUI service successful!");
		t = srv.response.pp_time;
		for(int i=0; i<4; i++) {
			(*pos)[i][0] = srv.response.pp.at(i).pos.at(0);
			(*pos)[i][1] = srv.response.pp.at(i).pos.at(1);
			(*pos)[i][2] = srv.response.pp.at(i).pos.at(2);
		}
	
	}
	else {
		ROS_INFO("GUI service call failed!");
		return -1;
	}
	return 0;
}







int main(int argc, char** argv)
{
	ros::init(argc, argv, "main_node");
	ros::NodeHandle node;


	ros::ServiceClient client_CC = node.serviceClient<controller_manager_msgs::SwitchController>("/yantra/controller_manager/switch_controller");	//CC = Controller change
	ros::ServiceClient client_IK = node.serviceClient<yantra::InverseKinematics>("inverse_kinematics_server");	//IK = Inverse Kinematics
	ros::ServiceClient client_TG = node.serviceClient<yantra::TrajectoryGenerator>("trajectory_generator_server");	//TG = Trajectory Generator
	ros::ServiceClient client_PG = node.serviceClient<yantra::PathPointsGui>("/yantra_gui");	//PG = Path-points GUI
	ros::Publisher pub_jointvalue = node.advertise<std_msgs::Float64MultiArray>("/yantra/yantra_arm_controller/command", 1);
	
	ros::Duration wait_time_server(15);

	client_IK.waitForExistence(wait_time_server);
	client_TG.waitForExistence(wait_time_server);
	client_CC.waitForExistence(wait_time_server);
	

	controller_manager_msgs::SwitchController controller_change_srv;
	std::vector<std::string> pos_controller	({"yantra_arm_controller"});
	std::vector<std::string> vel_controller	({"link_one_vel_controller","link_two_vel_controller","link_three_vel_controller","link_four_vel_controller","link_five_vel_controller"}); 
	
	
	double pos[4][3];			//4 = passing_points
	std::vector<double> time;
	int ret = call_gui(&client_PG, time, &pos);	
	if(ret == -1) {
			ROS_INFO("Could not get path points from GUI. Exiting!");
			return -1;
	}



	double q_init[] = {0, 0, 0, 0, 0};
	double q_j_value[passing_points][5];
	double q_j_velocity[passing_points][5] = {0};
	double q_j_accel[passing_points][5] = {0};
	double trajcoeffs[passing_points-1][5];
	yantra::JointValues waypoint_joint_space[passing_points];
	array3d coeff_a;

	

	/*
	 * Applying Inverse Kinematics to path points 
	 */
	for (int i=0; i<passing_points; i++)
	{
		int err = -5;
		int attempt = 0;
		while(err != 0 && attempt<3) {
			err = apply_IK(&client_IK, &pos[i][0], &q_init[0], &q_j_value[i][0]);
			attempt += 1;
		}
	}



	/*
	 * Just Printing the values the Joint Angles for 6 points
	 */
	std::cout << "Printing values of joint angles for given points" << std::endl;
	for(int i=0; i<passing_points; i++) {
		for(int j=0; j<5; j++) {
			std::cout << q_j_value[i][j] << " , ";
		}
		std::cout << std::endl;
	}




	/*
	 * Commanding Arm to go the first path-point from where the velocity controller will start
	 */
	std::vector<double> home_pos(q_j_value[0], q_j_value[0]+sizeof(q_j_value[0])/sizeof(q_j_value[0][0]));
	std_msgs::Float64MultiArray home_pos_pub_msg;
	home_pos_pub_msg.data = home_pos;
	pub_jointvalue.publish(home_pos_pub_msg);
	
	char ans;
	std::cout << "Is Home Position set?" << std::endl;
	std::cin >> ans;


	/*
	 * Generating Trajectory. Specifically calculating the path-segment coefficients
	 */
	for(int i=0; i<passing_points; i++)
	{
		waypoint_joint_space[i] = make_JointValues(&q_j_value[i][0], &q_j_velocity[i][0], &q_j_accel[i][0]);
	}
	
	int err = trajectory_generator(&client_TG, &waypoint_joint_space[0], time, coeff_a);
	if(err != 0) {
		std::cout << "No Trajectory coefficient was generated!" << std::endl;
		return 0;
	}


	/*
	 *
	// Applying coefficient to get the the position values
	for (int i=1; i<5; i++)  //waypoints or time
	{
		double diff = (time[i] - time[i-1])/10;
		for(int j=0; j<10; j++)
		{
				double t = time[i] + j*diff;
				std::vector<double> tmp;
				for (int k=0; k<5; k++) 	//for each joint
				{
					double pt = (coeff_a[i][k][0]*(std::pow(t,3)))+(coeff_a[i][k][1]*(std::pow(t,2)))+(coeff_a[i][k][2]*(t))+(coeff_a[i][k][3]);
					tmp.push_back(pt);
				}
				std::vector<double> pos_point = end_effector_position(&tmp[0]);
				vis_point.push_back(pos_point);
		}
	}
	*
	*/
	//double one_pt = (coeff_a[i][0][0]*(std::pow(t,3)))+(coeff_a[i][0][1]*(std::pow(t,2)))+(coeff_a[i][0][2]*(t))+(coeff_a[i][0][3]);
	


	controller_change_srv.request.start_controllers = vel_controller;
	controller_change_srv.request.stop_controllers = pos_controller;
	controller_change_srv.request.strictness = 2;
	controller_change_srv.request.start_asap = true;
	controller_change_srv.request.timeout = 0.0;

	/*
	 * Changin thr controller from Position COntroller to Velocity Controller.
	 * If successful, start the timer and publish the joint velocity as defined in timer callback function
	 */
	
	Publish_Timer _timer(node, coeff_a, time);
	if (client_CC.call(controller_change_srv)) {
			std::cout << "controller_change successful" << std::endl;

			_timer.start(node);			//Timer will stop automatically whwn time_count has reached end time (sec) 
	}
	
	ros::spin();
}


