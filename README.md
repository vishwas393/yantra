# YANTRA
## 6-DOF Serial Robotic Arm


This is a project based on the course "Advanced Robotic Kinematics and Dynamics" in masters program Robotic Systems Engineering at RWTH aachen university. 

![Movement GIF](https://github.com/vishwas393/yantra/blob/controller_switching/misc/movement.gif?raw=true "movement_gif")


ROS Version: Noetic </br>
Gazebo Version: 11.11.0

To run the application, launch
```sh
roslaunch yantra yantra_launch.launch algo:=imposed_velocity
```
</br>
A GUI will be launched where user must provide 4 path points and respective time (first time should be non-zero) for each point. User also need to confirm in newly opened terminal whether the home-position(1st point in GUI) is set in Gazebo.
</br></br>

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

3. **Trajectory Generator node**: This node performs the cubic polynomial joint trajectory. The execution of the algorithm depends on the argument provided. The two option are `approach:=imposed_velocity` and `approach:=accel_continuity`. The equation for joint angles is as shown in the below given equation. This node returns the coefficient of the equation.  <br/>
      ```math
      q_{i,k}(t) = a_{i,k,0} + a_{i,k,1}t + a_{i,k,2}t^{2} + a_{i,k,3}t^{3}
      ```
      ```math
      	i = i^{th} \text{ joint}
      ```
      ```math
      	k = k^{th} \text{ path segment}
      ```
       
	1. **Imposed Velocity**: This approach has a heuristic function which calculates the velocities of the intermidiate path-points. Here, User provides 4 path-points and hence, total 3 path-segment are generated. The start-point and end-point velocities are set to 0 by default. The velocities for rest of the points are calculated according to the below given function. The trajectory will not be intuitive because there is no orienttation defined for end-position as well as this is a joint space trajectory planner which focuses on path-points rather than path itself. 
        ```math
            \displaylines{\dot{q}_{i,k} = \begin{cases}0 & if sgn(v_{i,k}) \neq sgn(v_{i,k+1})\\\frac{1}{2}(v_{i,k} + v_{i,k+1}) & if sgn(v_{i,k}) = sgn(v_{i,k+1})\end{cases}}
	    ```
	    ```math
        
		\displaylines{v_{i,k} = \frac{(q_{i,k} - q_{i,k-1})} {(t_{i,k} - t_{i,k-1})}}
		\displaylines{k=1,...,N-1}
        ```
	

	2. **Acceleration Continuity**: For this approach, two virtual path-points are added to the system and continuity upto accelaration is calculated. (implemented but not yet working)
	
![Traj Eq](https://github.com/vishwas393/yantra/blob/controller_switching/misc/trajectory_image.png?raw=true "trajectory_equation")

The output of the trajectory planner can also visualised in MATLAB (matlab files in `misc` directory) via providing trajectory coefficient which gets printed via ROS node in terminal. Find the video of whole application [here].

[ROS controller]: http://wiki.ros.org/ros_control#Controllers
[here]: https://youtu.be/qAU0fLBAawA

