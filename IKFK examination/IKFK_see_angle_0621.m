clc; clear; close all;
load('kinematic_v2-1.mat');
load('inverse_kinematic_v2.mat');

%% Generate CIRCLE trajectory
center_y = 0;
center_z = -80;
R = 12;
X_fixed = 26.086;
theta_circ = linspace(0, 60, 60);
Z_circ = center_z - theta_circ;
Y_circ = center_y  * ones(size(Z_circ));
X_circ = X_fixed * ones(size(Z_circ));

target_points = [X_circ; Y_circ; Z_circ]';

%% Initialize FK returning point
P_fk = zeros(4, 60);
Q_fk = zeros(4, 60);
theta_list1 = zeros(3, 60);
theta_list2 = zeros(3, 60);

%% IK-FK execution
for i = 1:size(target_points, 1)
    point = target_points(i, :);
    theta_a_n = double(subs(theta_a, [x, y, z], point));
    theta_b_n = double(subs(theta_b, [x, y, z], point));
    theta_c1_n = double(subs(theta_c_1, [x, y, z], point));
    theta_c2_n = double(subs(theta_c_2, [x, y, z], point));
    theta_k_n = double(subs(theta_k, [x, y, z], point));

    % IK 
    theta_best_1 = [theta_a_n, theta_b_n, theta_c1_n];    
    theta_best_2 = [theta_a_n, theta_b_n, theta_c2_n]; 
    theta_list1(:,i) = [theta_a_n; theta_b_n; theta_c1_n];
    theta_list2(:,i) = [theta_a_n; theta_b_n; theta_c2_n];


end

save('IK_angle-v1.mat','theta_list1', 'theta_list2');
