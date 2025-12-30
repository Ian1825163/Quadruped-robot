clc; clear; close all;
% =========================================================================
% Main Refactored Script for Robot Leg Kinematics (v2 - FIXED)
%
% This script combines the functionality of four files AND fixes a
% critical continuity bug found in the original IK angle generation.
%
% 1. inverse_kinematics_v2.m (Symbolic IK)
% 2. kinematics_v2.m (Symbolic FK)
% 3. IKFK_see_angle_0621.m (IK Calculation)
% 4. leg_simulation_fromIK_1.m (FK Animation)
%
% * FIX (v2): Added continuity selection logic in "Section 4"
%   to ensure the 'theta_c' angles fed to the FK are continuous.
% =========================================================================

%% 1. Define All Physical Parameters
% All parameters are defined once in this struct.
params.L1 = 26.086; % (From inverse_kinematics_v2.m)
params.L2 = 80;     % (From inverse_kinematics_v2.m)
params.L3 = 80;     % (From inverse_kinematics_v2.m)
params.L4 = 20;     % (From inverse_kinematics_v2.m)
params.h = 28.5;    % (From kinematics_v2.m & inverse_kinematics_v2.m)
params.r = 22;      % (From kinematics_v2.m & inverse_kinematics_v2.m)
params.R3 = 20;     % (From kinematics_v2.m)
params.R4 = 80;     % (From kinematics_v2.m)

%% 2. Generate Analytical Functions (One-Time Cost)
% This step replaces running the separate symbolic scripts.
% It calls a local function to create fast, numerical function handles.
fprintf('Generating kinematic functions from symbolic equations...\n');
[IK_fcns, FK_fcns] = generate_kinematic_functions(params);
fprintf('Functions generated successfully.\n');

%% 3. Generate Target Trajectory
% This section is from IKFK_see_angle_0621.m
center_y = 0;
center_z = -80;
R = 12;
X_fixed = 26.086;
theta_circ = linspace(-20, 30, 50);
Z_circ = center_z - theta_circ;
Y_circ = center_y  * ones(size(Z_circ));
X_circ = X_fixed * ones(size(Z_circ));

target_points = [X_circ; Y_circ; Z_circ]; % Use (3 x N)
num_points = size(target_points, 2);

%% 4. Run Inverse Kinematics (with Continuity Fix)
% This replaces the main loop in IKFK_see_angle_0621.m
% *** THIS SECTION IS NOW FIXED ***
fprintf('Calculating continuous IK angles for trajectory...\n');

angles_list_continuous = zeros(3, num_points); % [alp; bet; gamm]
last_theta_c = []; % Persistent state for theta_c selection

for i = 1:num_points
    pos = target_points(:, i);
    
    % Calculate common angles (a, b)
    theta_a_n = IK_fcns.f_a(pos(1), pos(2), pos(3));
    theta_b_n = IK_fcns.f_b(pos(1), pos(2), pos(3));
    
    % Calculate *both* possible roots for theta_c
    theta_c1_n = IK_fcns.f_c1(pos(1), pos(2), pos(3));
    theta_c2_n = IK_fcns.f_c2(pos(1), pos(2), pos(3));

    % --- NEW: Root Selection for IK (theta_c) ---
    % This logic ensures the chosen angle is continuous
    chosen_theta_c = 0;
    if isempty(last_theta_c)
        % For the first point, just pick one (e.g., c1)
        chosen_theta_c = theta_c1_n;
    else
        % Pick the root that is closest to the last chosen one
        err1 = abs(theta_c1_n - last_theta_c);
        err2 = abs(theta_c2_n - last_theta_c);
        
        % Handle angle wrap-around (e.g., 3.14 vs -3.14)
        err1 = min(err1, 2*pi - err1);
        err2 = min(err2, 2*pi - err2);

        if err1 <= err2
            chosen_theta_c = theta_c1_n;
        else
            chosen_theta_c = theta_c2_n;
        end
    end
    
    % Store the chosen angle for the next iteration
    last_theta_c = chosen_theta_c;
    
    % Store the full continuous angle set
    angles_list_continuous(1, i) = theta_a_n;
    angles_list_continuous(2, i) = theta_b_n;
    angles_list_continuous(3, i) = chosen_theta_c;
end
fprintf('IK calculation complete.\n');

% Clear the persistent variable in the FK state calculator
% This ensures a clean start for the animation
clear calculate_FK_state;

%% 5. Run Forward Kinematics Simulation (Animation)
% This section replaces leg_simulation_fromIK_1.m
% It now uses the *continuous* angle list.
fprintf('Starting animation...\n');
figure('Name', 'Refactored Leg Simulation (v2 - Fixed)', 'NumberTitle', 'off');
set(gcf, 'Position', [100, 100, 800, 600]);

trajectory_history = zeros(3, num_points);
plot_handles = []; % Handles for plot objects

for i = 1:num_points
    % 1. Calculate FK
    % We pass the continuous angles to the FK calculator
    [E_pos, plot_points] = calculate_FK_state(angles_list_continuous(:, i), params, FK_fcns);
    
    % Store history for trajectory plotting
    trajectory_history(:, i) = E_pos;
    
    % 2. Draw the leg
    % Delete old plot objects
    if ~isempty(plot_handles)
        delete(plot_handles);
    end
    
    % Plot the new state
    plot_handles = plot_leg_state(plot_points, trajectory_history(:, 1:i));
    
    % 3. Pause for animation
    pause(0.05);
end

fprintf('Animation complete.\n');


%% 
%% ===================================================================
%% LOCAL FUNCTIONS (Helper functions)
%% ===================================================================

function [IK_fcns, FK_fcns] = generate_kinematic_functions(p)
    % This function generates all necessary numerical functions 
    % from the symbolic equations.
    
    % --- Part 1: Inverse Kinematics (from inverse_kinematics_v2.m) ---
    syms x y z;
    l = sqrt(x^2+y^2+z^2);                                                                                                                                              
    d = sqrt(l^2-p.L1^2);
    
    % theta_k (knee angle)
    theta_k = acos((p.L1^2+p.L2^2+p.L3^2-l^2)/(2*p.L2*p.L3));
    % theta_b (motor 2)
    theta_b = pi/2 - (asin(y/d) + acos((p.L2^2+d^2-p.L3^2)/(2*p.L2*d)));
    % theta_a (motor 1)
    theta_a = atan2(x,abs(z)) + acos(p.L1/ l) - pi/2;
    
    % theta_c (motor 3)
    A = 2*p.r*(p.L4*cos(theta_k)+p.L2);
    B = 6*p.r;
    C = p.r^2 + (p.L2 + p.L4*cos(theta_k))^2 + 9 + (p.L4*sin(theta_k) - p.h)^2 - p.L3^2;
    theta_c_1 = atan2(-A*B + sqrt((A*B)^2-(A^2-C^2)*(B^2-C^2)), A^2-C^2);
    theta_c_2 = atan2(-A*B - sqrt((A*B)^2-(A^2-C^2)*(B^2-C^2)), A^2-C^2);

    % Convert symbolic IK expressions to fast numerical functions
    vars_in = [x; y; z];
    IK_fcns.f_a = matlabFunction(theta_a, 'Vars', vars_in);
    IK_fcns.f_b = matlabFunction(theta_b, 'Vars', vars_in);
    IK_fcns.f_c1 = matlabFunction(theta_c_1, 'Vars', vars_in);
    IK_fcns.f_c2 = matlabFunction(theta_c_2, 'Vars', vars_in);
    
    % --- Part 2: Forward Kinematics (from kinematics_v2.m) ---
    syms alp bet gamm;
    
    % Transformation matrices
    T1 = [cos(alp) 0 -sin(alp) 0;
          0 1 0 0;
          sin(alp) 0 cos(alp) 0;
          0 0 0 1];
    T2 = [1 0 0 0;
          0 cos(bet) sin(bet) 0;
          0 -sin(bet) cos(bet) 0;
          0 0 0 1];
          
    % Four-bar linkage symbolic angles
    a = p.r*sin(gamm);
    R1 = sqrt(a^2+p.h^2);
    R2 = sqrt(p.R4^2-(p.r*cos(gamm)-3)^2); % R4=80
    theta1 = atan2(p.h, a);
    A_fk = 2*R1*p.R3*sin(theta1);
    B_fk = 2*R1*p.R3*cos(theta1)-2*p.R3*p.R4;
    C_fk = R1^2-R2^2+p.R3^2+p.R4^2-2*R1*p.R4*cos(theta1);
    theta4_1 = atan2(-A_fk*B_fk + sqrt(C_fk^2*(A_fk^2+B_fk^2-C_fk^2)), A_fk^2-C_fk^2);
    theta4_2 = atan2(-A_fk*B_fk - sqrt(C_fk^2*(A_fk^2+B_fk^2-C_fk^2)), A_fk^2-C_fk^2);

    % Convert symbolic FK expressions to fast numerical functions
    FK_fcns.f_T1 = matlabFunction(T1, 'Vars', alp);
    FK_fcns.f_T2 = matlabFunction(T2, 'Vars', bet);
    FK_fcns.f_th4_1 = matlabFunction(theta4_1, 'Vars', gamm);
    FK_fcns.f_th4_2 = matlabFunction(theta4_2, 'Vars', gamm);
end


% =========================================================================
function [E_pos, plot_points] = calculate_FK_state(theta, p, FK_fcns)
    % This function calculates all point positions for plotting.
    % It contains the root-selection logic from leg_simulation_fromIK_1.m
    
    % Input angles
    alp_n = theta(1);
    bet_n = theta(2);
    gamm_n = theta(3);

    % --- Four-Bar Linkage Root Selection ---
    % Use the fast numerical functions
    theta4_1_n = FK_fcns.f_th4_1(gamm_n);
    theta4_2_n = FK_fcns.f_th4_2(gamm_n);

    % Check points for error metric
    B_local_check = [p.r*cos(gamm_n); p.r*sin(gamm_n); p.h; 1];
    C1_1_local = [3; p.R4 + p.R3 * cos(theta4_1_n); p.R3 * sin(theta4_1_n); 1];
    C1_2_local = [3; p.R4 + p.R3 * cos(theta4_2_n); p.R3 * sin(theta4_2_n); 1];
    
    % Calculate error for both solutions (distance should be 80)
    error1 = abs(norm(B_local_check(1:3) - C1_1_local(1:3)) - p.L3); % p.L3 = 80
    error2 = abs(norm(B_local_check(1:3) - C1_2_local(1:3)) - p.L3); % p.L3 = 80
    
    % Use a persistent variable to remember the last choice
    % This prevents the linkage from "jumping" between solutions
    persistent last_choice
    if isempty(last_choice)
        last_choice = 1; % set initial choice
    end
    
    % Default choice
    if error1 <= error2
        choice = 1; % theta4_1
    else
        choice = 2; % theta4_2
    end
    
    % Hysteresis/Thresholding to maintain continuity
    threshold = 0.4; % (from original file)
    if last_choice == 1
        if error2 < error1 - threshold
            choice = 2; % Only switch if error is significantly better
        else
            choice = 1; % Stick with the current solution
        end
    else % last_choice == 2
        if error1 < error2 - threshold
            choice = 1; % Only switch if error is significantly better
        else
            choice = 2; % Stick with the current solution
        end
    end
    
    % Set final chosen angle for the four-bar linkage
    if choice == 1
        theta4_n = theta4_1_n;
    else
        theta4_n = theta4_2_n;
    end
    
    last_choice = choice; % Store choice for next iteration

    % --- Final FK Position Calculation ---
    
    % Get motor transformation matrices
    T1_n = FK_fcns.f_T1(alp_n);
    T2_n = FK_fcns.f_T2(bet_n);
    
    % Global transformation for the main leg
    T_global = T1_n * T2_n;
    
    % Global transformation for the sub-assembly (four-bar linkage)
    % This frame is offset by L1 along the new x-axis
    T_sub_assembly = T_global * [1 0 0 p.L1; 0 1 0 0; 0 0 1 0; 0 0 0 1];
    
    % --- Define all points for plotting ---
    
    % Main joints in global frame
    plot_points.O = [0;0;0]; % Motor 1 (Origin)
    M_global = T_global * [p.L1; 0; 0; 1];
    plot_points.M = M_global(1:3); % Motor 2
    K_global = T_global * [p.L1; 0; p.h; 1];
    plot_points.K = K_global(1:3); % Motor 3
    
    % Four-bar linkage points (in local frame, then transformed)
    B_local = [p.r*cos(gamm_n); p.r*sin(gamm_n); p.h; 1];
    C_local = [0; p.R4+p.R3*cos(theta4_n); p.R3*sin(theta4_n); 1];
    C1_local = [3; p.R4+p.R3*cos(theta4_n); p.R3*sin(theta4_n); 1];
    D_local = [0; p.R4; 0; 1];
    E_local = D_local - [0; p.R4*cos(theta4_n); p.R4*sin(theta4_n); 0];
    
    % Transform local sub-assembly points to global frame
    B_global = T_sub_assembly * B_local;
    plot_points.B = B_global(1:3);
    
    C_global = T_sub_assembly * C_local;
    plot_points.C = C_global(1:3);
    
    C1_global = T_sub_assembly * C1_local;
    plot_points.C1 = C1_global(1:3);
    
    D_global = T_sub_assembly * D_local;
    plot_points.D = D_global(1:3);
    
    E_global = T_sub_assembly * E_local;
    plot_points.E = E_global(1:3); % This is the end-effector
    
    % Return end-effector position
    E_pos = plot_points.E;
end


% =========================================================================
function plot_handles = plot_leg_state(p, trajectory)
    % This function handles all plotting.
    % It receives a struct 'p' containing all points.
    
    hold on; % Start plotting
    
    plot_handles = []; % Store handles for deletion
    
    % Plot links
    h = plot3([p.O(1) p.M(1)],[p.O(2) p.M(2)],[p.O(3),p.M(3)],'k','linewidth',1.5); % O to M
    plot_handles = [plot_handles; h];
    h = plot3([p.K(1) p.M(1)],[p.K(2) p.M(2)],[p.K(3),p.M(3)],'k','linewidth',1.5); % K to M
    plot_handles = [plot_handles; h];
    
    % Four-bar linkage
    % (Note: Plotting B-C1 as the coupler based on error check)
    h = plot3([p.K(1) p.B(1)],[p.K(2) p.B(2)],[p.K(3),p.B(3)],'r','linewidth',2); % Input link
    plot_handles = [plot_handles; h];
    h = plot3([p.B(1) p.C1(1)],[p.B(2) p.C1(2)],[p.B(3),p.C1(3)],'k','linewidth',1); % Coupler
    plot_handles = [plot_handles; h];
    h = plot3([p.C1(1) p.C(1)],[p.C1(2) p.C(2)],[p.C1(3),p.C(3)],'k','linewidth',1); % Part of rigid triangle
    plot_handles = [plot_handles; h];
    h = plot3([p.M(1) p.D(1)],[p.M(2) p.D(2)],[p.M(3),p.D(3)],'k','linewidth',1.5); % Thigh link (M to D)
    plot_handles = [plot_handles; h];
    h = plot3([p.C(1) p.E(1)],[p.C(2) p.E(2)],[p.C(3),p.E(3)],'k','linewidth',1.5); % Shank link (C to E)
    plot_handles = [plot_handles; h];
    
    % Plot joints
    all_points = [p.O, p.M, p.K, p.B, p.C, p.C1, p.D, p.E];
    h = scatter3(all_points(1,:), all_points(2,:), all_points(3,:), 30, 'k', 'filled');
    plot_handles = [plot_handles; h];
    
    % Highlight end-effector
    h = scatter3(p.E(1), p.E(2), p.E(3), 50, 'b', 'filled');
    plot_handles = [plot_handles; h];

    % Plot trajectory history
    if ~isempty(trajectory)
        h = plot3(trajectory(1,:), trajectory(2,:), trajectory(3,:), ...
            'b.-', 'LineWidth', 1);
        plot_handles = [plot_handles; h];
    end

    % Set fixed plot properties
    x_a = [-15,40];
    y_a = [-15,100];
    z_a = [-140, 30]; % Adjusted z-limit to see full trajectory
    xlim([x_a(1),x_a(2)+5]);
    ylim([y_a(1),y_a(2)+5]);
    zlim([z_a(1),z_a(2)+5]);
    
    % Draw axes
    h = plot3([x_a(1) x_a(2)], [0 0], [0 0], 'k:','Linewidth',1);
    plot_handles = [plot_handles; h];
    h = plot3([0 0],[y_a(1) y_a(2)],[0 0], 'k:','Linewidth',1);
    plot_handles = [plot_handles; h];
    h = plot3([0 0],[0 0], [z_a(1),z_a(2)], 'k:','Linewidth',1);
    plot_handles = [plot_handles; h];

    grid on;
    axis equal; 
    view(3); % 3D view
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    
    hold off; % Done plotting
end