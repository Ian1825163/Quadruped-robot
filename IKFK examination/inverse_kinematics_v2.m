clc; clear; close all;
L1 = 26.086;
L2 = 80; %根到腿
L3 = 80; %桿長
L4 = 20; 
h = 28.5;
r = 22;
syms x y z;
l = sqrt(x^2+y^2+z^2);                                                                                                                                              
d = sqrt(l^2-L1^2);
theta_k = acos((L1^2+L2^2+L3^2-l^2)/(2*L2*L3)); %the knee angle is between 0~pi, so acos is valid
theta_b = pi/2 - (asin(y/d) + acos((L2^2+d^2-L3^2)/(2*L2*d)));
theta_a = atan2(x,abs(z)) + acos(L1/ l) - pi/2; %pi/2 > theta_a > -pi/2

%小腿平面
A = 2*r*(L4*cos(theta_k)+L2);
B = 6*r;
C = r^2 + (L2 + L4*cos(theta_k))^2 + 9 + (L4*sin(theta_k) - h)^2 - L3^2;
theta_c_1 = atan2(-A*B + sqrt((A*B)^2-(A^2-C^2)*(B^2-C^2)), A^2-C^2);
theta_c_2 = atan2(-A*B - sqrt((A*B)^2-(A^2-C^2)*(B^2-C^2)), A^2-C^2);


save('inverse_kinematic_v2.mat','theta_a', 'theta_b', ...
    'theta_c_1','theta_c_2', 'theta_k', 'L1', 'L2', 'L3', 'h', 'r', 'd', 'l', 'x','y','z');
save('inverse_kinematic_v2.mat','theta_a', 'theta_b', ...
    'theta_c_1','theta_c_2', 'theta_k', 'L1', 'L2', 'L3', 'h', 'r', 'd', 'l', 'x','y','z');