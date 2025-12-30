    clc; clear; close all;
    
    load('kinematic_v2-1.mat')
    load('IK_angle-v1.mat')

    
    %v = VideoWriter('robot_leg_animation_0620_1_6.mp4', 'MPEG-4');
    %v.FrameRate = 20;
    %open(v);
    %%  plot the result
    trajectory = zeros(3, size(theta_list1, 2));  % 儲存腳尖軌跡

    for i = 1:size(theta_list2, 2)
        [end_pos] = plot_linkage(theta_list1(:, i)', trajectory(:, 1:i-1)); % 傳入已有軌跡
        trajectory(:, i) = end_pos(1:3);  % 儲存目前末端點（腳尖）位置
        %frame = getframe(gcf);  
        %writeVideo(v, frame); 
        pause(0.05);
        clf;         
    end
    %close(v);
    %% function fordward kinematic
    function [E_plot] = plot_linkage(theta, trajectory_so_far)
        load('kinematic_v2-1_plot.mat')
    
        %theta4_n=double(subs(theta4,[gamm],[theta(3)]));
        
        theta4_1_n = double(subs(theta4_1, gamm, theta(3)));
        theta4_2_n = double(subs(theta4_2, gamm, theta(3)));

        T1_n = double(subs(T1, alp, theta(1)));
        T2_n = double(subs(T2, bet, theta(2)));
        
        B=[r*cos(theta(3)); r*sin(theta(3)); h; 1];
        C1_1 = [3; R4 + R3 * cos(theta4_1_n); R3 * sin(theta4_1_n); 1];
        C1_2 = [3; R4 + R3 * cos(theta4_2_n); R3 * sin(theta4_2_n); 1];
        L1 = norm(B(1:3) - C1_1(1:3));
        L2 = norm(B(1:3) - C1_2(1:3));
        persistent last_choice
    
        if isempty(last_choice)
            last_choice = 1; % set initial choice
        end
        
        error1 = abs(L1-80);
        error2 = abs(L2-80);
        
        if error1 <= error2
            choice = 1; % theta4_1
        else
            choice = 2; % theta4_2
        end
        
        threshold = 0.4; % adjustable
        
        if last_choice == 1
            if error2 < error1 - threshold
                choice = 2; % 只有在真的誤差小很多才換
            else
                choice = 1; % 否則堅持原本的
            end
        else
            if error1 < error2 - threshold
                choice = 1;
            else
                choice = 2;
            end
        end
        
        %set next decision
        if choice == 1
            theta4_n = theta4_1_n;
        else
            theta4_n = theta4_2_n;
        end
        
        last_choice = choice;

        %% positions
        O=[0;0;0;1]; % motor1
        M = T1_n*T2_n*[L1; 0; 0; 1]; %motor2
        K = T1_n*T2_n*[L1; 0; h; 1]; %motor3
        %B=[r*cos(theta(3)); r*sin(theta(3)); d; 1];
        C=[0;R4+R3*cos(theta4_n); R3*sin(theta4_n); 1];
        C1=[3;R4+R3*cos(theta4_n); R3*sin(theta4_n); 1];
        D=[0;R4;0;1];
        E= D - [0; R4*cos(theta4_n); R4*sin(theta4_n);0];
        
        O_plot = T1_n*T2_n*(O + [L1; 0; 0; 0]);
        C_plot = T1_n*T2_n*(C + [L1; 0; 0; 0]);
        C1_plot = T1_n*T2_n*(C1 + [L1; 0; 0; 0]);
        D_plot = T1_n*T2_n*(D + [L1; 0; 0; 0]);
        E_plot = T1_n*T2_n*(E + [L1; 0; 0; 0]);
        B_plot = T1_n*T2_n*(B + [L1; 0; 0; 0]);
        
    
        plot3([O(1) M(1)],[O(2) M(2)],[O(3),M(3)],'k','linewidth',1);hold on; %motor1 to motor2
        plot3([K(1) M(1)],[K(2) M(2)],[K(3),M(3)],'k','linewidth',1);hold on; %motor1 to motor2

        plot3([K(1) B_plot(1)],[K(2) B_plot(2)],[K(3),B_plot(3)],'k','linewidth',1);hold on;
        plot3([B_plot(1) C_plot(1)],[B_plot(2) C_plot(2)],[B_plot(3),C_plot(3)],'k','linewidth',1);hold on;
        plot3([C1_plot(1) C_plot(1)],[C1_plot(2) C_plot(2)],[C1_plot(3),C_plot(3)],'k','linewidth',1);hold on;
        plot3([O_plot(1) D_plot(1)],[O_plot(2) D_plot(2)],[O_plot(3),D_plot(3)],'k','linewidth',1);hold on;
        plot3([C_plot(1) E_plot(1)],[C_plot(2) E_plot(2)],[C_plot(3),E_plot(3)],'k','linewidth',1);hold on;
        

        scatter3(O_plot(1),O_plot(2),O_plot(3),'k');
        scatter3(B_plot(1),B_plot(2),B_plot(3),'k');
        scatter3(C_plot(1),C_plot(2),C_plot(3),'k');
        scatter3(C1_plot(1),C1_plot(2),C1_plot(3),'k');
        scatter3(D_plot(1),D_plot(2),D_plot(3),'k');
        scatter3(E_plot(1),E_plot(2),E_plot(3),'k');
    
        x_a = [-15,40];
        y_a = [-15,100];
        z_a = [-100 30];
        xlim([x_a(1),x_a(2)+5]);
        ylim([y_a(1),y_a(2)+5]);
        zlim([z_a(1),z_a(2)+5]);
        plot3([x_a(1) x_a(2)], [0 0], [0 0], 'k','Linewidth',1);
        plot3([0 0],[y_a(1) y_a(2)],[0 0], 'k','Linewidth',1);
        plot3([0 0],[0 0], [z_a(1),z_a(2)], 'k','Linewidth',1);
        text(x_a(2)+1, 0, 0, 'X', 'Color', 'r');
        text(0, y_a(2)+1, 0, 'Y', 'Color', 'g');
        text(0, 0, z_a(2)+1, 'Z', 'Color', 'b');
        disp(theta4_n);
    
        grid on;
        axis equal; 
    
        % 畫腳尖歷史軌跡（藍色線）
        if ~isempty(trajectory_so_far)
            plot3(trajectory_so_far(1,:), trajectory_so_far(2,:), trajectory_so_far(3,:), ...
                'b.-', 'LineWidth', 1.5);
        end

    end