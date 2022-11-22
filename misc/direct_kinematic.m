function pos = direct_kinematic(d1, d2, d3,d4, d5, d6)
    pos = zeros(3,1);
%     pos(1,:) = 56*cos(d1) + 70*sin(d4)*(cos(d1)*cos(d2)*cos(d3) - cos(d1)*sin(d2)*sin(d3)) + 56*sin(d5)*(cos(d1)*cos(d2)*sin(d3) + cos(d1)*cos(d3)*sin(d2)) + 56*cos(d5)*(sin(d1)*sin(d4) + cos(d4)*(cos(d1)*cos(d2)*cos(d3) - cos(d1)*sin(d2)*sin(d3))) + 205*cos(d1)*cos(d2) - 70*cos(d4)*sin(d1) + 56*cos(d1)*cos(d2)*cos(d3) + 167*cos(d1)*cos(d2)*sin(d3) + 167*cos(d1)*cos(d3)*sin(d2) - 56*cos(d1)*sin(d2)*sin(d3);
%     pos(2,:) = 56*sin(d1) + 70*sin(d4)*(cos(d2)*cos(d3)*sin(d1) - sin(d1)*sin(d2)*sin(d3)) + 56*sin(d5)*(cos(d2)*sin(d1)*sin(d3) + cos(d3)*sin(d1)*sin(d2)) + 56*cos(d5)*(cos(d4)*(cos(d2)*cos(d3)*sin(d1) - sin(d1)*sin(d2)*sin(d3)) - cos(d1)*sin(d4)) + 70*cos(d1)*cos(d4) + 205*cos(d2)*sin(d1) + 56*cos(d2)*cos(d3)*sin(d1) + 167*cos(d2)*sin(d1)*sin(d3) + 167*cos(d3)*sin(d1)*sin(d2) - 56*sin(d1)*sin(d2)*sin(d3);
%     pos(3,:) = 205*sin(d2) + 167*sin(d2)*sin(d3) + 70*sin(d4)*(cos(d2)*sin(d3) + cos(d3)*sin(d2)) + 56*sin(d5)*(sin(d2)*sin(d3) - cos(d2)*cos(d3)) - 167*cos(d2)*cos(d3) + 56*cos(d2)*sin(d3) + 56*cos(d3)*sin(d2) + 56*cos(d4)*cos(d5)*(cos(d2)*sin(d3) + cos(d3)*sin(d2)) + 283;

    pos(1,:) = 56*cos(d1) + 104*sin(d4)*(cos(d1)*cos(d2)*cos(d3) - cos(d1)*sin(d2)*sin(d3)) + 205*cos(d1)*cos(d2) - 104*cos(d4)*sin(d1) + 56*cos(d1)*cos(d2)*cos(d3) + 167*cos(d1)*cos(d2)*sin(d3) + 167*cos(d1)*cos(d3)*sin(d2) - 56*cos(d1)*sin(d2)*sin(d3);
    pos(2,:) = 56*sin(d1) + 104*sin(d4)*(cos(d2)*cos(d3)*sin(d1) - sin(d1)*sin(d2)*sin(d3)) + 104*cos(d1)*cos(d4) + 205*cos(d2)*sin(d1) + 56*cos(d2)*cos(d3)*sin(d1) + 167*cos(d2)*sin(d1)*sin(d3) + 167*cos(d3)*sin(d1)*sin(d2) - 56*sin(d1)*sin(d2)*sin(d3);
    pos(3,:) = 205*sin(d2) + 167*sin(d2)*sin(d3) + 104*sin(d4)*(cos(d2)*sin(d3) + cos(d3)*sin(d2)) - 167*cos(d2)*cos(d3) + 56*cos(d2)*sin(d3) + 56*cos(d3)*sin(d2) + 283;


end