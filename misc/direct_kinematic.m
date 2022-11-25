function pos = direct_kinematic(d1, d2, d3,d4, d5)
    pos = zeros(3,1);
    pos(1,:) = 104*cos(d5)*(cos(d1)*cos(d3)*cos(d2 + pi/2) - cos(d1)*sin(d3)*sin(d2 + pi/2)) + 56*sin(d5)*(cos(d1)*cos(d3)*cos(d2 + pi/2) - cos(d1)*sin(d3)*sin(d2 + pi/2)) + 56*cos(d5)*(sin(d1)*sin(d4) - cos(d4)*(cos(d1)*cos(d3)*sin(d2 + pi/2) + cos(d1)*cos(d2 + pi/2)*sin(d3))) - 104*sin(d5)*(sin(d1)*sin(d4) - cos(d4)*(cos(d1)*cos(d3)*sin(d2 + pi/2) + cos(d1)*cos(d2 + pi/2)*sin(d3))) + 205*cos(d1)*cos(d2 + pi/2) + 291*cos(d1)*cos(d3)*cos(d2 + pi/2) - 291*cos(d1)*sin(d3)*sin(d2 + pi/2);
    pos(2,:) = 205*cos(d2 + pi/2)*sin(d1) + 104*cos(d5)*(cos(d3)*cos(d2 + pi/2)*sin(d1) - sin(d1)*sin(d3)*sin(d2 + pi/2)) + 56*sin(d5)*(cos(d3)*cos(d2 + pi/2)*sin(d1) - sin(d1)*sin(d3)*sin(d2 + pi/2)) - 56*cos(d5)*(cos(d4)*(cos(d3)*sin(d1)*sin(d2 + pi/2) + cos(d2 + pi/2)*sin(d1)*sin(d3)) + cos(d1)*sin(d4)) + 104*sin(d5)*(cos(d4)*(cos(d3)*sin(d1)*sin(d2 + pi/2) + cos(d2 + pi/2)*sin(d1)*sin(d3)) + cos(d1)*sin(d4)) + 291*cos(d3)*cos(d2 + pi/2)*sin(d1) - 291*sin(d1)*sin(d3)*sin(d2 + pi/2);
    pos(3,:) = 205*sin(d2 + pi/2) + 291*cos(d3)*sin(d2 + pi/2) + 291*cos(d2 + pi/2)*sin(d3) + 104*cos(d5)*(cos(d3)*sin(d2 + pi/2) + cos(d2 + pi/2)*sin(d3)) + 56*sin(d5)*(cos(d3)*sin(d2 + pi/2) + cos(d2 + pi/2)*sin(d3)) - 56*cos(d4)*cos(d5)*(sin(d3)*sin(d2 + pi/2) - cos(d3)*cos(d2 + pi/2)) + 104*cos(d4)*sin(d5)*(sin(d3)*sin(d2 + pi/2) - cos(d3)*cos(d2 + pi/2)) + 283;
end