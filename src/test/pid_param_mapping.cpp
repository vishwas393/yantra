#include "ros/ros.h"
#include "control_msgs/JointControllerState.h"
#include "std_msgs/Float64.h"



class PID_tuner_param_configur
{
		public:
		PID_tuner_param_configur()
		{
			pub_setpoint = nh.advertise<std_msgs::Float64>("/pid_param/setpoint",5);
			pub_currstate = nh.advertise<std_msgs::Float64>("/pid_param/state",5);

		}
		
		void start()
		{
			sub_state = nh.subscribe<control_msgs::JointControllerState>("/yantra/link_two_vel_controller/state", 5, &PID_tuner_param_configur::callback_fn, this);
			ros::spin();
		}
	
		private:
		ros::NodeHandle nh;
		ros::Publisher pub_setpoint;
		ros::Publisher pub_currstate;
		ros::Subscriber sub_state;

		std_msgs::Float64 setpoint;	
		std_msgs::Float64 state;	

	
		void callback_fn(const control_msgs::JointControllerState::ConstPtr& msg)
		{
			setpoint.data 		= msg->set_point;
			state.data 			= msg->process_value;

			pub_setpoint.publish(setpoint);
			pub_currstate.publish(state);
		}

};


int main(int argc, char** argv)
{
	ros::init(argc, argv, "pid_param_mapping_node");

	PID_tuner_param_configur pid_param_configurer;
	pid_param_configurer.start();
	//ros::spin();
}
