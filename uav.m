% 시간 설정 변경 금지
clear; clc; close all;
total_time = 20; 
dt = 0.005;      
time = 0:dt:total_time; % Time 
state = zeros(6, 1);
initial_state = state;


path_history = zeros(3, length(time));
path_history(:,1) = state(1:3);

%% 강체 설정

fuselage_len = 2.0; 
wing_span = 2.5;    
fuselage = [-fuselage_len/2, 0, 0; fuselage_len/2, 0, 0]';
wings = [0, -wing_span/2, 0; 0, wing_span/2, 0]';
tail = [-fuselage_len/2, 0, 0; -fuselage_len/2, 0, -0.5]';
uav_body = [fuselage, wings, tail];



% 시각화 설정 변경 금지
figure('Name', '2020253091 김호중', 'NumberTitle', 'off', 'Color', 'w');
ax = gca;
hold(ax, 'on');
grid(ax, 'on');
axis(ax, 'equal');
view(ax, 30, 20); % Set initial viewing angle
xlabel(ax, 'X [m]');
ylabel(ax, 'Y [m]');
zlabel(ax, 'Z [m]');
title(ax, '2020253091 김호중', 'NumberTitle');
set(ax, 'XLim', [-50 50], 'YLim', [-50 50], 'ZLim', [0 25]);

%젬민이의 도움
h_fuselage = plot3(ax, NaN, NaN, NaN, 'r-', 'LineWidth', 3);
h_wings = plot3(ax, NaN, NaN, NaN, 'k-', 'LineWidth', 2.5);
h_tail = plot3(ax, NaN, NaN, NaN, 'b-', 'LineWidth', 2);
h_trajectory = plot3(ax, path_history(1,1), path_history(2,1), path_history(3,1), 'g:', 'LineWidth', 1.5);


% 루프
for i = 2:length(time)
    t = time(i);
    

    if t < 5
        linear_vel = [3; 0; 1.5]; 
    elseif t < 12
        linear_vel = [4; 0; 0]; 
    else
        linear_vel = [3.5; 0; -0.5]; 
    end
    
    if t > 5 && t < 15
        % Coordinated turn
        angular_vel = [0.1; 0.05; 0.3]; % Roll, pitch and yaw
    else
        angular_vel = [0; 0; 0];
    end
    
    % 기구한 계산 회전행렬
   
    phi = state(4);
    theta = state(5);
    psi = state(6);
    
    R_x = [1, 0, 0; 0, cos(phi), -sin(phi); 0, sin(phi), cos(phi)];
    R_y = [cos(theta), 0, sin(theta); 0, 1, 0; -sin(theta), 0, cos(theta)];
    R_z = [cos(psi), -sin(psi), 0; sin(psi), cos(psi), 0; 0, 0, 1];
    R = R_z * R_y * R_x;
    

    if abs(cos(theta)) < 1e-6
        W = [1, sin(phi)*tan(theta + 1e-6), cos(phi)*tan(theta + 1e-6);
             0, cos(phi),                   -sin(phi);
             0, sin(phi)/cos(theta + 1e-6), cos(phi)/cos(theta + 1e-6)];
    else
        W = [1, sin(phi)*tan(theta), cos(phi)*tan(theta);
             0, cos(phi),            -sin(phi);
             0, sin(phi)/cos(theta), cos(phi)/cos(theta)];
    end
    
    pos_dot = R * linear_vel;
    angle_dot = W * angular_vel;
    state_dot = [pos_dot; angle_dot];
    
    state = state + state_dot * dt;
    
    path_history(:, i) = state(1:3);
    
    % 시각화 업데이터
    rotated_body = R * uav_body;
    translated_body = rotated_body + state(1:3);
    
    
    set(h_fuselage, 'XData', translated_body(1, 1:2), 'YData', translated_body(2, 1:2), 'ZData', translated_body(3, 1:2));
    set(h_wings, 'XData', translated_body(1, 3:4), 'YData', translated_body(2, 3:4), 'ZData', translated_body(3, 3:4));
    set(h_tail, 'XData', translated_body(1, 5:6), 'YData', translated_body(2, 5:6), 'ZData', translated_body(3, 5:6));
    set(h_trajectory, 'XData', path_history(1, 1:i), 'YData', path_history(2, 1:i), 'ZData', path_history(3, 1:i));
    
    drawnow limitrate; 
end
