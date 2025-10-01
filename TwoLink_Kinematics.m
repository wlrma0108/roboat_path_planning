
function TwoLink_Kinematics()

    L = [1.0, 0.8]; % L1, L2길이    
    target_pos = [0.6; 1.2]; 
    
    [q_solutions, ~ ] = inverse_kinematics(L, target_pos);
    q_elbow_up = q_solutions(1,:)'; 
    
    J = calculate_jacobian(L, q_elbow_up);
    
    % 결과 요약 출력
    disp('목표 위치 x, y:'); disp(target_pos');
    disp('자코비안'); disp(J);
 
    
    animate_trajectory(L, [0;0], q_elbow_up, '궤적');
    
end

% 정기구학
function pos = forward_kinematics(L, q)
    pos = [L(1)*cos(q(1)) + L(2)*cos(q(1)+q(2));
           L(1)*sin(q(1)) + L(2)*sin(q(1)+q(2))];
end
% 역기구학
function [q_sols, is_ok] = inverse_kinematics(L, pos_d)
    L1=L(1); L2=L(2); x=pos_d(1); y=pos_d(2);
    dist_sq = x^2 + y^2;
    if dist_sq > (L1+L2)^2 || dist_sq < (L1-L2)^2, q_sols=[]; is_ok=false; return; end
    
    cos_q2 = (dist_sq - L1^2 - L2^2) / (2 * L1 * L2);
    q2_down = acos(max(-1, min(1, cos_q2)));
    q2_up = -q2_down;
    
    q1_down = atan2(y,x) - atan2(L2*sin(q2_down), L1+L2*cos(q2_down));
    q1_up   = atan2(y,x) - atan2(L2*sin(q2_up), L1+L2*cos(q2_up));
    
    q_sols = [q1_up, q2_up; q1_down, q2_down];
    is_ok = true;
end
% 자코바
function J = calculate_jacobian(L, q)
    L1=L(1); L2=L(2); q1=q(1); q2=q(2); s1=sin(q1); c1=cos(q1); s12=sin(q1+q2); c12=cos(q1+q2);
    J = [-L1*s1-L2*s12, -L2*s12;
          L1*c1+L2*c12,  L2*c12];
end


function animate_trajectory(L, q_start, q_end, fig_title)
    figure('Name', fig_title);
    max_reach = sum(L);
    
    for t = linspace(0, 1, 100)
        q_current = q_start*(1-t) + q_end*t;
        pos_j1 = forward_kinematics([L(1),0], q_current);
        pos_ee = forward_kinematics(L, q_current);
        
        cla;
        plot([0, pos_j1(1), pos_ee(1)], [0, pos_j1(2), pos_ee(2)], 'o-', 'LineWidth', 3, 'MarkerSize', 8, 'MarkerFaceColor', 'k');
        grid on; axis equal; axis([-max_reach, max_reach, -max_reach, max_reach]*1.2);
        drawnow;
    end
end