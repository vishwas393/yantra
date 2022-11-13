close all;
% coeff [joint][path_segment][a_i]
coeff_j1 = [[0.0187215  -0  -2.77556e-17  0.169234];  
[-0.0540301  0.43651  -0.87302  0.751247];  
[0.036347  -0.648016  3.46508  -5.03289 ]; 
[0.171008  -3.07192  18.0417  -34.3191 ]; 
[-0.172047 5.1614  -51.614  171.638 ]]; 

coeff_j2 = [[0.00105647  -0  1.73472e-17  -0.430424] ; 
[0.0116498  -0.06356  0.0805304  -0.421991];  
[-0.0241107  0.365566  -1.59606  1.70702];  
[0.146103  -2.69827  16.422  -32.8694];  
[-0.134698  4.04095  -40.4095  134.243]];  

coeff_j3 = [[-0.00676448  0  6.93889e-17  -0.90122];  
[0.00118898  -0.0477207  0.117589  -1.00914];  
[0.0189167  -0.260453  0.915987  -1.93359];  
[-0.19694  3.62496  -21.9696  42.1299];  
[0.183599  -5.50796  55.0796  -184.592 ]]; 

coeff_j4 = [[-0.00636581  0  1.38778e-17  0.305474];  
[0.0247284  -0.186565  0.299723  0.203536];  
[-0.0124612  0.25971  -1.50894  2.67791];  
[-0.254816  4.6221  -27.4944  53.8935];  
[0.248915  -7.46744  74.6744  -247.637]];  

coeff_j5 = [[-0.000446624  0  2.89121e-19  -0.00108236];  
[-0.00467194  0.0253519  -0.0302305  -0.00822636];  
[0.00925445  -0.141765  0.621697  -0.833358];  
[-0.0468535  0.868179  -5.30562  10.4919];  
[0.0427176  -1.28153  12.8153  -42.7545]]; 

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
cl = ['r','g','b','c','m'];
c = 1;
for i=1:1:99
    if floor(i/20) ~= floor((i-1)/20)
        c = c + 1;
        disp(c);
    end
    plot3(positions(i,1), positions(i,2), positions(i,3), '*', 'Color',cl(c));
    hold on;
end

%%
figure; plot(1:50, positions(:,1));
figure; plot(1:50, positions(:,2));
figure; plot(1:50, positions(:,3));