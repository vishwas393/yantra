clear all;
syms d1 d2 d3 d4 d5 d6

pos_x = 205*cos(d1)*cos(d2 + pi/2) + 291*cos(d1)*cos(d3)*cos(d2 + pi/2) - 291*cos(d1)*sin(d3)*sin(d2 + pi/2);
pos_y = 205*cos(d2 + pi/2)*sin(d1) + 291*cos(d3)*cos(d2 + pi/2)*sin(d1) - 291*sin(d1)*sin(d3)*sin(d2 + pi/2);
pos_z = 205*sin(d2 + pi/2) + 291*cos(d3)*sin(d2 + pi/2) + 291*cos(d2 + pi/2)*sin(d3) + 283;


%%
J_x_d1 = diff(pos_x,d1)
J_x_d2 = diff(pos_x,d2)
J_x_d3 = diff(pos_x,d3)
J_x_d4 = diff(pos_x,d4)
J_x_d5 = diff(pos_x,d5)
J_x_d6 = diff(pos_x,d6)

J_y_d1 = diff(pos_y,d1)
J_y_d2 = diff(pos_y,d2)
J_y_d3 = diff(pos_y,d3)
J_y_d4 = diff(pos_y,d4)
J_y_d5 = diff(pos_y,d5)
J_y_d6 = diff(pos_y,d6)

J_z_d1 = diff(pos_z,d1)
J_z_d2 = diff(pos_z,d2)
J_z_d3 = diff(pos_z,d3)
J_z_d4 = diff(pos_z,d4)
J_z_d5 = diff(pos_z,d5)
J_z_d6 = diff(pos_z,d6)