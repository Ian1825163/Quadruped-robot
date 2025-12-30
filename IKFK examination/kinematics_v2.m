clc; clear; close all;
h = 28.5;
R3 = 20;
R4 = 80;
r = 22;
%gamm : angle of motor3(deg)
%alp  : angle of motor1
%beta : angle of motor2
syms alp bet gamm;
a = r*sin(gamm);
R1 = sqrt(a^2+h^2);
R2 = sqrt(80^2-(r*cos(gamm)-3)^2);
theta1 = atan2(h, a);
A = 2*R1*R3*sin(theta1);
B = 2*R1*R3*cos(theta1)-2*R3*R4;
C = R1^2-R2^2+R3^2+R4^2-2*R1*R4*cos(theta1);
theta4_1 = atan2(-A*B + sqrt(C^2*(A^2+B^2-C^2)), A^2-C^2);
theta4_2 = atan2(-A*B - sqrt(C^2*(A^2+B^2-C^2)), A^2-C^2);



T1 = [cos(alp) 0 -sin(alp) 0;
    0 1 0 0;
    sin(alp) 0 cos(alp) 0;
    0 0 0 1];
T2 = [1 0 0 0;
    0 cos(bet) sin(bet) 0;
    0 -sin(bet) cos(bet) 0;
    0 0 0 1]; %use subs() aferward
save('kinematic_v2-1.mat','T1', 'T2', ...
    'h','R1', 'R2', 'R3', 'R4', 'r', 'a' , 'theta4_1','theta4_2', 'alp','bet','gamm');
save('kinematic_v2-1_plot.mat','T1', 'T2', ...
    'h','R1', 'R2', 'R3', 'R4', 'r', 'a' , 'theta4_1','theta4_2','alp','bet','gamm');
