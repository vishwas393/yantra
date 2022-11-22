%% FINAL TRANSFORMATION MATRIX
clear all;

syms d1 d2 d3 d4 d5;
T_01  = transformation_matrix(0, 187,     0,   0);
T_12  = transformation_matrix(d1, 96, pi/2,  0);
T_23 = transformation_matrix(d2+pi/2, 0, 0, 205);
T_33d = transformation_matrix(d3, 0, pi, 124);
T_3d4 = transformation_matrix(-pi/2, 0, -pi/2, 0);
T_45 = transformation_matrix(d4, 167, pi/2, 0);

T_55d = transformation_matrix(d5, 0, -pi/2, 0);
T_5d6 = transformation_matrix(0, 104, 0, 112);


T = T_01*T_12*T_23*T_33d*T_3d4*T_45; 


%% DEBUG PURPOSE
VT1 = round(vpa(subs(T_01, {d1,d2,d3,d4,d5}, {0,0,0,0,0})),3);
VT2 = round(vpa(subs(T_12, {d1,d2,d3,d4,d5}, {0,0,0,0,0})),3);
VT3 = round(vpa(subs(T_23, {d1,d2,d3,d4,d5}, {0,0,0,0,0})),3);
VT3d = round(vpa(subs(T_33d, {d1,d2,d3,d4,d5}, {0,0,0,0,0})),3);
VT4 = round(vpa(subs(T_3d4, {d1,d2,d3,d4,d5}, {0,0,0,0,0})),3);
VT5 = round(vpa(subs(T_45, {d1,d2,d3,d4,d5}, {0,0,0,0,0})),3);

VT01 = round(vpa(subs(T_01, {d1,d2,d3,d4,d5}, {0,0,0,0,0})),3);
VT012 = round(vpa(subs(T_01*T_12, {d1,d2,d3,d4,d5}, {0,0,0,0,0})),3);
VT0123 = round(vpa(subs(T_01*T_12*T_23, {d1,d2,d3,d4,d5}, {0,0,0,0,0})),3);
VT01233d = round(vpa(subs(T_01*T_12*T_23*T_33d, {d1,d2,d3,d4,d5}, {0,0,0,0,0})),3);
VT0123d4 = round(vpa(subs(T_01*T_12*T_23*T_3d4, {d1,d2,d3,d4,d5}, {0,0,0,0,0})),3);
VT012345 = round(vpa(subs(T_01*T_12*T_23*T_33d*T_3d4*T_45, {d1,d2,d3,d4,d5}, {0,0,0,0,0})),3);

%% CALCULATING END EFFECTOR POSITION VECTOR

r1  = T_01(:,4);
r2  = T_01*T_12(:,4);
r3  = T_01*T_12*T_23(:,4);
r3d = T_01*T_12*T_23*T_33d(:,4);
r4  = T_01*T_12*T_23*T_33d*T_3d4(:,4);
r5  = T_01*T_12*T_23*T_33d*T_3d4*T_45(:,4);
r5d = T_01*T_12*T_23*T_33d*T_3d4*T_45*T_55d(:,4);
r6  = T_01*T_12*T_23*T_33d*T_3d4*T_45*T_55d*T_5d6(:,4);

r = r6;

%% DEBUG PURPOSE
vr1 = vpa(subs(r1, {d1,d2,d3,d4,d5}, {0,0,0,0,0}));
vr2 = vpa(subs(r2, {d1,d2,d3,d4,d5}, {0,0,0,0,0}));
vr3 = vpa(subs(r3, {d1,d2,d3,d4,d5}, {0,0,0,0,0}));
vr3d = vpa(subs(r3d, {d1,d2,d3,d4,d5}, {0,0,0,0,0}));
vr4 = vpa(subs(r4, {d1,d2,d3,d4,d5}, {0,0,0,0,0}));
vr5 = vpa(subs(r5, {d1,d2,d3,d4,d5}, {0,0,0,0,0}));
vr5d = vpa(subs(r5d, {d1,d2,d3,d4,d5}, {0,0,0,0,0}));
vr6 = vpa(subs(r6, {d1,d2,d3,d4,d5}, {0,0,0,0,0}));

vr = round([vr1 vr2 vr3 vr4 vr5 vr6], 3)