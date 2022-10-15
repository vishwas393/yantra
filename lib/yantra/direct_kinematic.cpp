#include <cmath>
#include <vector>
/*
 * 	Brief: Predefined end effector position vector.
         Retrieved from Transformation matrix T_0_5.
	Input : array of joint angles in radians
	Output: Position vector of End Effector 
	*/

std::vector<double> end_effector_position(std::vector<double> d)
{
    double d1 = d[0];
    double d2 = d[1];
    double d3 = d[2];
    double d4 = d[3];
    double d5 = d[4];
	std::vector<double> t;
    
	double t1 = 56.0*cos(d1) - 56.0*cos(d3)*(- 1.0*cos(d1)*cos(d2)) + 56.0*sin(d5)*(- 1.0*sin(d3)*(- 1.0*cos(d1)*cos(d2)) + cos(d3)*(cos(d1)*sin(d2))) - 56.0*cos(d5)*(cos(d4)*(cos(d3)*(- 1.0*cos(d1)*cos(d2)) + sin(d3)*(cos(d1)*sin(d2))) - 1.0*sin(d4)*(sin(d1))) - 170.0*sin(d3)*(- 1.0*cos(d1)*cos(d2)) + 170.0*cos(d3)*(cos(d1)*sin(d2)) - 56.0*sin(d3)*(cos(d1)*sin(d2)) + 200.0*cos(d1)*cos(d2);

    double t2 = 56.0*sin(d1) - 56.0*sin(d5)*(- 1.0*cos(d3)*(sin(d1)*sin(d2)) - 1.0*sin(d3)*(cos(d2)*sin(d1))) + 56.0*cos(d5)*(cos(d4)*(cos(d3)*(cos(d2)*sin(d1)) - 1.0*sin(d3)*(sin(d1)*sin(d2))) - 1.0*sin(d4)*(cos(d1))) + 170.0*cos(d3)*(sin(d1)*sin(d2)) + 56.0*cos(d3)*(cos(d2)*sin(d1)) - 56.0*sin(d3)*(sin(d1)*sin(d2)) + 170.0*sin(d3)*(cos(d2)*sin(d1)) + 200.0*cos(d2)*sin(d1);

    double t3 = 200.0*sin(d2) + 170.0*sin(d2)*sin(d3) + 56.0*cos(d5)*(cos(d4)*(cos(d2)*sin(d3) + cos(d3)*sin(d2))) + 56.0*sin(d5)*(sin(d2)*sin(d3) - 1.0*cos(d2)*cos(d3)) - 170.0*cos(d2)*cos(d3) + 56.0*cos(d2)*sin(d3) + 56.0*cos(d3)*sin(d2) + 280.0;

	t.push_back(t1);
	t.push_back(t2);
	t.push_back(t3);
    return t;
}

