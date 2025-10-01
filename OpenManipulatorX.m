
function OpenManipulatorX()
    %% 기본 파라미터
    dh_params = [0, -pi/2, 0.077, 0; 0.130, 0, 0, -pi/2; 0.124, 0, 0, 0; 0, 0, 0, 0];
    ee_offset = [0.126; 0; 0];

    target_pos = [0.54; 0.3; 0.56];
    initial_q = deg2rad([0; 20; 30; 40]);
    q_sol = inverse_kinematics(dh_params, target_pos, ee_offset, initial_q);
    
    disp('목표 위치 [x,y,z]:'); disp(target_pos');
    animate_trajectory(dh_params, zeros(4,1), q_sol, ee_offset, '궤적 애니메이션');
end

% 행렬식 정리 dh
function T = dh_transform(a, alpha, d, theta)
    T = [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
         sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
         0,           sin(alpha),             cos(alpha),            d;
         0,           0,                      0,                     1];
end

% 기구학
function [T_final, T_intermediate] = forward_kinematics(dh, q, ee_offset)
    T_intermediate = cell(size(dh, 1) + 1, 1);
    T_intermediate{1} = eye(4);
    for i = 1:size(dh, 1)
        T_intermediate{i+1} = T_intermediate{i} * dh_transform(dh(i,1), dh(i,2), dh(i,3), q(i) + dh(i,4));
    end
    T_final = T_intermediate{end} * [eye(3), ee_offset; 0 0 0 1];
end

% 자코비안 
function J = calculate_jacobian(T_intermediate, ee_offset)
    temp_transform = T_intermediate{end} * [ee_offset; 1];
    p_end = temp_transform(1:3);
    J = zeros(6, length(T_intermediate)-1);
    for i = 1:length(T_intermediate)-1
        z_im1 = T_intermediate{i}(1:3, 3);
        p_im1 = T_intermediate{i}(1:3, 4);
        J(1:3, i) = cross(z_im1, p_end - p_im1);
        J(4:6, i) = z_im1;
    end
end
% 역기구학
function q = inverse_kinematics(dh, p_d, ee, q0)
    q = q0; alpha = 0.2; lambda = 0.01;
    for i = 1:100
        [T_curr, T_int] = forward_kinematics(dh, q, ee);
        p_curr = T_curr(1:3, 4);
        err = p_d - p_curr;
        if norm(err) < 1e-4, break; end
        J_pos = calculate_jacobian(T_int, ee);
        J_pos = J_pos(1:3, :);
        delta_q = J_pos' * ((J_pos*J_pos' + lambda^2*eye(3)) \ err);
        q = q + alpha * delta_q;
    end
end

 
function animate_trajectory(dh, q_start, q_end, ee_offset, fig_title)
    figure('Name', fig_title);
    for t = linspace(0, 1, 100)
        q_curr = q_start*(1-t) + q_end*t;
        [~, T_int] = forward_kinematics(dh, q_curr, ee_offset);
        
        points_cell = cellfun(@(T) T(1:3, 4), T_int, 'UniformOutput', false);
        points = horzcat(points_cell{:});
        
        temp_transform = T_int{end} * [ee_offset; 1];
        points(:, end+1) = temp_transform(1:3);
        
        cla;
        plot3(points(1,:), points(2,:), points(3,:), 'o-', 'Color', [0 0.4470 0.7410], 'LineWidth', 3, 'MarkerSize', 8, 'MarkerFaceColor','k');
        grid on; axis equal; axis([-0.4 0.4 -0.4 0.4 0 0.5]); view(30, 20);
        drawnow;
    end
end