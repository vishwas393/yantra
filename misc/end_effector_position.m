function p = end_effector_position(d1, d2, d3, d4, d5)
    
	x = 56.0*cos(d1) - 56.0*cos(d3)*(- 1.0*cos(d1)*cos(d2)) + 56.0*sin(d5)*(- 1.0*sin(d3)*(- 1.0*cos(d1)*cos(d2)) + cos(d3)*(cos(d1)*sin(d2))) - 56.0*cos(d5)*(cos(d4)*(cos(d3)*(- 1.0*cos(d1)*cos(d2)) + sin(d3)*(cos(d1)*sin(d2))) - 1.0*sin(d4)*(sin(d1))) - 170.0*sin(d3)*(- 1.0*cos(d1)*cos(d2)) + 170.0*cos(d3)*(cos(d1)*sin(d2)) - 56.0*sin(d3)*(cos(d1)*sin(d2)) + 200.0*cos(d1)*cos(d2);
    y = 56.0*sin(d1) - 56.0*sin(d5)*(- 1.0*cos(d3)*(sin(d1)*sin(d2)) - 1.0*sin(d3)*(cos(d2)*sin(d1))) + 56.0*cos(d5)*(cos(d4)*(cos(d3)*(cos(d2)*sin(d1)) - 1.0*sin(d3)*(sin(d1)*sin(d2))) - 1.0*sin(d4)*(cos(d1))) + 170.0*cos(d3)*(sin(d1)*sin(d2)) + 56.0*cos(d3)*(cos(d2)*sin(d1)) - 56.0*sin(d3)*(sin(d1)*sin(d2)) + 170.0*sin(d3)*(cos(d2)*sin(d1)) + 200.0*cos(d2)*sin(d1);
    z = 200.0*sin(d2) + 170.0*sin(d2)*sin(d3) + 56.0*cos(d5)*(cos(d4)*(cos(d2)*sin(d3) + cos(d3)*sin(d2))) + 56.0*sin(d5)*(sin(d2)*sin(d3) - 1.0*cos(d2)*cos(d3)) - 170.0*cos(d2)*cos(d3) + 56.0*cos(d2)*sin(d3) + 56.0*cos(d3)*sin(d2) + 280.0;
    p = [x;y;z];
end