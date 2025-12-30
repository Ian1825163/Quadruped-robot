clc; clear; close all;
% =========================================================================
% Main Script to Plot Robot Leg Workspace (v5 - Surface Plot)
%
% Method: Plot a separate 2D surface (using 'surf') for each
%         discrete angle of motor 1 (alp).
%
% * FIX (v5): Replaced scatter3 with surf to "connect the faces"
%   as requested.
% * NOTE: The motor 2 (bet) range is based on the previous script.
%   Please verify the sign/range is correct based on your findings.
% =========================================================================


%% 1. Define All Physical Parameters
% All parameters are defined once in this struct.
params.L1 = 26.086;
params.L2 = 80;
params.L3 = 80;
params.L4 = 20;
params.h = 28.5;
params.r = 22;
params.R3 = 20;
params.R4 = 80;

%% 2. Generate Analytical Functions (One-Time Cost)
% This calls the local function to create fast, numerical function handles.
fprintf('Generating kinematic functions from symbolic equations...\n');
[~, FK_fcns] = generate_kinematic_functions(params);
fprintf('Functions generated successfully.\n');

%% 3. Define Motor Angle Ranges and Resolution
% Define the resolution (step size) for the loops.
deg_step_alp = 1;   % Step size for motor 1 (alp) in degrees
deg_step_bet = 2;   % Step size for motor 2 (bet) in degrees
deg_step_gamm = 2;  % Step size for motor 3 (gamm) in degrees

% Create angle vectors in RADIANS
alp_range = deg2rad(0 : deg_step_alp : 10);
bet_range = deg2rad(0 : deg_step_bet : 90);
gamm_range = deg2rad(-36 : deg_step_gamm : 70);

% *** USER NOTE ***
% You mentioned motor 2's sign was wrong.
% Please double-check if 'bet_range' should be, for example:
% The code below will use whatever range you define here.

num_a = length(alp_range);
num_b = length(bet_range);
num_g = length(gamm_range);
total_points = num_a * num_b * num_g;
fprintf('Total points to calculate: %d (%d surfaces)\n', total_points, num_a);

% Pre-allocate a matrix to store ALL points for min/max calculation
all_workspace_points = nan(total_points, 3);
k_all = 1; % Global index for all_workspace_points

%% 4. Calculate and Plot Workspace Surfaces
fprintf('Calculating and plotting workspace surfaces...\n');
figure('Name', 'Robot Leg Workspace (Surface Plot)', 'NumberTitle', 'off');
set(gcf, 'Position', [100, 100, 900, 700]);
hold on; % Hold the plot for multiple surfaces

% Get a color map to color each 'alp' surface differently
colors = parula(num_a);

% Loop for Motor 1 (alp) - Each 'alp' is one surface
for i = 1:num_a
    alp = alp_range(i);
    fprintf('  Plotting surface %d/%d (alp = %.1f deg)...\n', i, num_a, rad2deg(alp));
    
    % Create X, Y, Z matrices for this specific 'alp' surface
    % Dimensions are (num_bet x num_gamm)
    X_surf = nan(num_b, num_g);
    Y_surf = nan(num_b, num_g);
    Z_surf = nan(num_b, num_g);
    
    % Loop for Motor 2 (bet)
    for j = 1:num_b
        bet = bet_range(j);
        
        % Loop for Motor 3 (gamm)
        for l = 1:num_g
            gamm = gamm_range(l);
            
            % Current angle vector
            theta = [alp; bet; gamm];
            
            % Calculate Forward Kinematics
            % The function returns NaN if no Z<0 solution exists
            [E_pos, ~] = calculate_FK_state(theta, params, FK_fcns);
            
            % Store X, Y, Z for the surface plot
            X_surf(j, l) = E_pos(1);
            Y_surf(j, l) = E_pos(2);
            Z_surf(j, l) = E_pos(3);
            
            % Store in the global list for min/max calculation
            all_workspace_points(k_all, :) = E_pos';
            k_all = k_all + 1;
        end
    end
    
    % Now we have the full (X,Y,Z) grid for this 'alp'
    % Plot it as a surface
    % 'surf' will automatically ignore NaN values
    surf(X_surf, Y_surf, Z_surf, ...
         'FaceColor', colors(i, :), ...
         'EdgeColor', 'none', ...
         'FaceAlpha', 0.8);
end

hold off;
fprintf('Calculation complete.\n');

%% 5. Plot Formatting
title('End-Effector Workspace');
xlabel('X');
ylabel('Y');
zlabel('Z');
grid on;
axis equal;
view(3); % 3D view
rotate3d on; % Allow interactive rotation

% Add a color bar to show the 'alp' angle mapping
colormap(colors);
c = colorbar;
c.Label.String = 'Motor 1 Angle (alp)';
% Set ticks to be in degrees
c.Ticks = linspace(0, 1, num_a);
c.TickLabels = sprintfc('%.0f deg', rad2deg(alp_range));


hold off;
fprintf('Calculation complete.\n');

%% 6. Display Numerical Range
% 'min' and 'max' functions with 'omitnan' flag
fprintf('\n--- Workspace Numerical Range (Z<0) ---\n');
min_X = min(all_workspace_points(:,1), [], 'omitnan');
max_X = max(all_workspace_points(:,1), [], 'omitnan');
min_Y = min(all_workspace_points(:,2), [], 'omitnan');
max_Y = max(all_workspace_points(:,2), [], 'omitnan');
min_Z = min(all_workspace_points(:,3), [], 'omitnan');
max_Z = max(all_workspace_points(:,3), [], 'omitnan');

fprintf('X-Range: %.2f to %.2f\n', min_X, max_X);
fprintf('Y-Range: %.2f to %.2f\n', min_Y, max_Y);
fprintf('Z-Range: %.2f to %.2f\n', min_Z, max_Z);
fprintf('---------------------------------\n');


%% 
%% ===================================================================
%% LOCAL FUNCTIONS (Unchanged from v4)
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
    % (Unchanged from v4 - Robust Z-Filter)
    
    % Input angles
    alp_n = theta(1);
    bet_n = theta(2);
    gamm_n = theta(3);

    % --- Four-Bar Linkage Root Selection ---
    theta4_1_n = FK_fcns.f_th4_1(gamm_n);
    theta4_2_n = FK_fcns.f_th4_2(gamm_n);

    % --- Final FK Position Calculation ---
    T1_n = FK_fcns.f_T1(alp_n);
    T2_n = FK_fcns.f_T2(bet_n);
    T_global = T1_n * T2_n;
    T_sub_assembly = T_global * [1 0 0 p.L1; 0 1 0 0; 0 0 1 0; 0 0 0 1];
    
    % --- Calculate Solution 1 (using theta4_1_n) ---
    E_local_1 = [0; p.R4; 0; 1] - [0; p.R4*cos(theta4_1_n); p.R4*sin(theta4_1_n); 0];
    E_global_1 = T_sub_assembly * E_local_1;
    
    % --- Calculate Solution 2 (using theta4_2_n) ---
    E_local_2 = [0; p.R4; 0; 1] - [0; p.R4*cos(theta4_2_n); p.R4*sin(theta4_2_n); 0];
    E_global_2 = T_sub_assembly * E_local_2;
    
    % --- NEW ROBUST Z-FILTER LOGIC ---
    
    % Check validity of both solutions
    sol1_is_valid = E_global_1(3) < 0;
    sol2_is_valid = E_global_2(3) < 0;
    
    final_theta4_n = 0; % (placeholder for plotting struct)
    
    if sol1_is_valid && sol2_is_valid
        % Both are valid (Z < 0), pick the lower (more negative) one
        if E_global_1(3) <= E_global_2(3)
            E_pos = E_global_1(1:3);
            final_theta4_n = theta4_1_n;
        else
            E_pos = E_global_2(1:3);
            final_theta4_n = theta4_2_n;
        end
    elseif sol1_is_valid
        % Only solution 1 is valid
        E_pos = E_global_1(1:3);
        final_theta4_n = theta4_1_n;
    elseif sol2_is_valid
        % Only solution 2 is valid
        E_pos = E_global_2(1:3);
        final_theta4_n = theta4_2_n;
    else
        % NEITHER solution is valid (both are Z >= 0)
        E_pos = [NaN; NaN; NaN]; % Return NaN
        final_theta4_n = theta4_1_n; % (doesn't matter)
    end

    % --- Define points for plotting (using the chosen theta4_n) ---
    % (This part is not strictly necessary for the workspace plot)
    plot_points.O = [0;0;0];
    M_global = T_global * [p.L1; 0; 0; 1];
    plot_points.M = M_global(1:3);
    K_global = T_global * [p.L1; 0; p.h; 1];
    plot_points.K = K_global(1:3);
    
    B_local = [p.r*cos(gamm_n); p.r*sin(gamm_n); p.h; 1];
    C_local = [0; p.R4+p.R3*cos(final_theta4_n); p.R3*sin(final_theta4_n); 1];
    C1_local = [3; p.R4+p.R3*cos(final_theta4_n); p.R3*sin(final_theta4_n); 1];
    D_local = [0; p.R4; 0; 1];
    
    B_global = T_sub_assembly * B_local;
    plot_points.B = B_global(1:3);
    C_global = T_sub_assembly * C_local;
    plot_points.C = C_global(1:3);
    C1_global = T_sub_assembly * C1_local;
    plot_points.C1 = C_global(1:3);
    D_global = T_sub_assembly * D_local;
    plot_points.D = D_global(1:3);
    
    plot_points.E = E_pos(1:3); % E_pos is already the chosen [x;y;z] or NaN
end