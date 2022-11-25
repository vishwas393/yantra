close all;
clear all;

% coeff [joint][path_segment][a_i]
coeff_j1 = [[0.0286562  -0  -0  0]; 
[-0.113104  0.850559  -1.70112  1.13408];  
[0.122702  -1.97911  9.61757  -13.9575];  
[-0.363379  6.77035  -41.6551  83.6912];  
[0.325124  -9.75373  97.5373  -324.82]];  


coeff_j2 = [[-0.000324295  0  5.78241e-19  0.230973];  
[-0.0134873  0.0789781  -0.107314  0.234992];
[0.0254498  -0.388267  1.72093  -2.09405]; 
[-0.149555  2.76182  -16.8072  33.4725 ]; 
[0.137917  -4.13751  41.3751  -137.614]]; 


coeff_j3 = [[0.00459786  -0  -9.25186e-17  -2.46759]; 
[0.100564  -0.575797  0.733867  -2.39987]; 
[-0.149765  2.42815  -11.1063  12.9187]; 
[0.0082109  -0.415413  4.52779  -12.6403]; 
[0.0363919  -1.09176  10.9176  -34.9013]]; 


coeff_j4 = [[-0.00431922  0  -0  0]; 
[0.010269  -0.0875291  0.151273  -0.0691351]; 
[-0.00344082  0.0769882  -0.516812  0.848355]; 
[-0.0704059  1.28236  -7.66876  14.8311];
[0.067897 -2.03691  20.3691  -67.8496]]; 

coeff_j5 = [[-0.0169434  0  9.25186e-18  -0.148139];  
[0.0279447  -0.269328  0.494654  -0.419237];
[-0.00375753  0.111098  -1.05602  1.72558]; 
[-0.0877903  1.62369  -9.89975  18.4858]; 
[0.0805465  -2.4164  24.164  -81.647]]; 

coeff = zeros(5, 5, 4);
coeff(1, :, :) = coeff_j1; 
coeff(2, :, :) = coeff_j2; 
coeff(3, :, :) = coeff_j3; 
coeff(4, :, :) = coeff_j4; 
coeff(5, :, :) = coeff_j5; 


cnt = 1;
positions = zeros(100, 3);
for t = 0:0.1:9.9
    path_seg = floor((floor(t))/2) + 1 ;
    q1 = coeff(1,path_seg,1)*t^3 + coeff(1,path_seg,2)*t^2 + coeff(1,path_seg,3)*t + coeff(1,path_seg,4);
    q2 = coeff(2,path_seg,1)*t^3 + coeff(2,path_seg,2)*t^2 + coeff(2,path_seg,3)*t + coeff(2,path_seg,4);
    q3 = coeff(3,path_seg,1)*t^3 + coeff(3,path_seg,2)*t^2 + coeff(3,path_seg,3)*t + coeff(3,path_seg,4);
    q4 = coeff(4,path_seg,1)*t^3 + coeff(4,path_seg,2)*t^2 + coeff(4,path_seg,3)*t + coeff(4,path_seg,4);
    q5 = coeff(5,path_seg,1)*t^3 + coeff(5,path_seg,2)*t^2 + coeff(5,path_seg,3)*t + coeff(5,path_seg,4);
 
    positions(cnt, :) = end_effector_position(q1, q2, q3, q4, q5);
    cnt = cnt + 1;
end

%%
cl = ['r','g','b','y','m'];
c = 1;
figure;

for i=1:1:99
    if floor(i/20) ~= floor((i-1)/20)
        c = c + 1;
    end
    plot3(positions(i,1), positions(i,2), positions(i,3), '*', 'Color',cl(c));
    hold on;
end
grid on;
axis([-600 600 -600 600 0 600])
%%
figure; plot(1:50, positions(:,1));
figure; plot(1:50, positions(:,2));
figure; plot(1:50, positions(:,3));