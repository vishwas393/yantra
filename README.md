# YANTRA
## 6-DOF Serial Robotic Arm


This is a project based on the course "Advanced Robotic Kinematics and Dynamics" in masters program Robotic Systems Engineering at RWTH aachen university. 

The project mainly consists of three nodes.
1. **Main node**: This is the main node the application. It first defines the path-points and initial joint positions, velocities and accelerations. It calls the **`service`**`inverse_kinematics_server` to calculate the joint values for given path-poinrs. After receiving the joint values, **`service`**`trajectory_generator_server` is called to generate the trajectory coefficient. At last, velocity for each joint is calculated using the trajecory coefficients and is published to [ROS controller] (*effort_controller/JointPositionGroupController* and *effort_controller/JointVelocityController*) <br/>

2. **Inverse Kinematics node**: This node performs the inverse kinematics for a given position in 3D-space as a ROS service. The main node sends the 4 path-points specified by the user(or hard-coded in this example) and this node returns joint-values for 5 joints of the arm.(6th joint is rotation for end-tool which is ignored for now.)
    - **`ROS service :`**`'inverse_kinematics_server'`
    - **`request :`**
        *   initial joint values (array of joint position, velocity and accelaration for each joint)
        *   position in 3D space (an array with 3 elements - X, Y, Z value)
    - **`response : `**
        *   Joint values (array of calculated joint position, velocity and accelaration for each joint)
	<br/>

3. **Trajectory Generator node**: This node performs the cubic polynomial joint trajectory. The equation for joint angles is as shown in the below given equation. The equation can also be understood from the image given below. The coefficient of the equation is then used to calculate the velocity. <br/>
       q<sub>i,k</sub>(t) = a<sub>i,k,3</sub>t<sup>3</sup> + a<sub>i,k,2</sub>t<sup>2</sup> + a<sub>i,k,1</sub>t<sup>1</sup> + a<sub>i,k,0</sub> <br/>
       - i : i<sup>th</sup> joint of the arm (total 5)
       - k : k<sup>th</sup> path segment (total 5. 4 user-defined path-points and 2 virtual points for continuity upto acceleration).

![Traj Eq](https://github.com/vishwas393/yantra/blob/controller_switching/misc/trajectory_image.png?raw=true "trajectory_equation")

The output of the trajectory planner is visualised in MATLAB (matlab file in `misc` directory) and the whole application is visualised in the Gazebo. The output of MATLAB is attached below as well as a short GIF. Find the video of whole application [here].(not uploaded yet) 

![MATLAB Plot](https://github.com/vishwas393/yantra/blob/controller_switching/misc/eepos_matlab_plot.jpg?raw=true "matlabplot")

![Movement GIF](https://github.com/vishwas393/yantra/blob/controller_switching/misc/movement.gif?raw=true "movement_gif")

[ROS controller]: http://wiki.ros.org/ros_control#Controllers
[here]: www.youtube.com
