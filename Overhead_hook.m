
clear; clc; close all;
% 시간
total_time = 3;
dt = 0.02;      
time = 0:dt:total_time;
l_leg = 0.9;       
l_torso = 0.6;    % 몸통
l_upper_arm = 0.3;% 위팔
l_forearm = 0.25; % 아래팔

%시각화 
fig = figure('Name', 'Overhead Hook Motion Simulation', 'NumberTitle', 'off', 'Color', 'w');
ax = gca;
hold(ax, 'on');
grid(ax, 'on');
axis(ax, 'equal');
view(ax, 45, 20); 
xlabel(ax, 'X [m]');
ylabel(ax, 'Y [m]');
zlabel(ax, 'Z [m]');
title(ax, 'Simplified Overhead Hook Kinematics');
set(ax, 'XLim', [-1.0, 1.0], 'YLim', [-1.0, 1.0], 'ZLim', [0, 2.2]);

% 각 링크
h_leg = plot3(ax, NaN, NaN, NaN, 'k-', 'LineWidth', 4);
h_torso = plot3(ax, NaN, NaN, NaN, 'b-', 'LineWidth', 4);
h_arm = plot3(ax, NaN, NaN, NaN, 'r-', 'LineWidth', 3);
h_joints = plot3(ax, NaN, NaN, NaN, 'ko', 'MarkerFaceColor', 'y', 'MarkerSize', 6);

legend('다리', '몸통', '팔', '관절', 'Location', 'northeast');

%루프
for i = 1:length(time)
    t = time(i);
    
    % --- 사용자가 창을 닫았을 때 오류 방지 ---
    if ~isvalid(fig)
        disp('시뮬레이션 창이 닫혀서 중단되었습니다.');
        break;
    end
    
    % 기구학 연산
    
    
    % 몸통 회전 
    theta_torso = - (pi/2.5) * sin((pi * t / total_time) * 1.5 - pi/2) - pi/2.5;

    % 어깨 움직임 과 펀치 
    theta_shoulder_pitch = (pi * 0.8) * sin((pi * t / total_time) - pi/2) + pi*0.7;
    
    theta_shoulder_roll = (pi/8) * (1 - cos(2 * pi * t / total_time));

    theta_elbow = (pi/2) * (1 - cos(2 * pi * t / total_time)) + pi/10;
    

    % 정방향
    p_base = [0; 0; 0];
    
    p_hip = p_base + [0; 0; l_leg];
    
    R_torso = [cos(theta_torso), -sin(theta_torso), 0; 
               sin(theta_torso), cos(theta_torso),  0; 
               0,                0,                 1]; % Z축(Yaw) 회전
    p_shoulder = p_hip + R_torso * [0; 0; l_torso];
    
    % 3. 팔꿈치
    R_shoulder_pitch = [cos(theta_shoulder_pitch), 0, sin(theta_shoulder_pitch);
                        0,                         1, 0;
                       -sin(theta_shoulder_pitch), 0, cos(theta_shoulder_pitch)]; 
    R_shoulder_roll = [1, 0, 0;
                       0, cos(theta_shoulder_roll), -sin(theta_shoulder_roll);
                       0, sin(theta_shoulder_roll), cos(theta_shoulder_roll)]; 
    
    % 몸통 회전  어깨 회전 
    R_shoulder = R_torso * R_shoulder_pitch * R_shoulder_roll;
    p_elbow = p_shoulder + R_shoulder * [l_upper_arm; 0; 0]; 
    
    %손목
    R_elbow = [cos(theta_elbow), -sin(theta_elbow), 0;
               sin(theta_elbow), cos(theta_elbow),  0;
               0,                0,                 1]; 
    
    % 어깨 회전 팔꿈치 회전 
    R_wrist = R_shoulder * R_elbow;
    p_wrist = p_elbow + R_wrist * [l_forearm; 0; 0];
    
    %애니메이션
    set(h_leg, 'XData', [p_base(1), p_hip(1)], 'YData', [p_base(2), p_hip(2)], 'ZData', [p_base(3), p_hip(3)]);
    set(h_torso, 'XData', [p_hip(1), p_shoulder(1)], 'YData', [p_hip(2), p_shoulder(2)], 'ZData', [p_hip(3), p_shoulder(3)]);
    set(h_arm, 'XData', [p_shoulder(1), p_elbow(1), p_wrist(1)], 'YData', [p_shoulder(2), p_elbow(2), p_wrist(2)], 'ZData', [p_shoulder(3), p_elbow(3), p_wrist(3)]);
    
    joint_positions = [p_hip, p_shoulder, p_elbow, p_wrist];
    set(h_joints, 'XData', joint_positions(1,:), 'YData', joint_positions(2,:), 'ZData', joint_positions(3,:));
    
    drawnow; 
end