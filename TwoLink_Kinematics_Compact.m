
function TwoLink_Kinematics_Compact()

    L = [1.0, 0.8]; % L1, L2길이
    
    fprintf('2RManipulator');
    
    target_pos = [0.6; 1.2]; 
    
    % 역기구학(IK) 계산
    [q_solutions, is_reachable] = inverse_kinematics(L, target_pos);
    assert(is_reachable, '목표 위치에 도달할 수 없습니다.');
    q_elbow_up = q_solutions(1,:)'; 
    
    % 정기구학(FK)으로 IK 결과 검증
    pos_fk_validation = forward_kinematics(L, q_elbow_up);
    
    J = calculate_jacobian(L, q_elbow_up);
    
    % 결과 요약 출력
    disp('목표 위치 [x, y]:'); disp(target_pos');
    disp('IK 해 (Elbow-Up) [q1, q2] (deg):'); disp(rad2deg(q_elbow_up'));
    disp('FK 검증 위치 [x, y]:'); disp(pos_fk_validation');
    disp('자코비안 '); disp(J);
 
    visualize_robot(L, q_elbow_up, target_pos, 'IK 최종 자세');
    
    animate_trajectory(L, [0;0], q_elbow_up, '궤적 애니메이션');
    
    fprintf('▶ 분석 및 시각화 완료.\n');
end


function pos = forward_kinematics(L, q)
    % 정기구학: q = [q1; q2] -> pos = [x; y]
    pos = [L(1)*cos(q(1)) + L(2)*cos(q(1)+q(2));
           L(1)*sin(q(1)) + L(2)*sin(q(1)+q(2))];
end

function [q_sols, is_ok] = inverse_kinematics(L, pos_d)
    % 역기구학: pos = [x; y] -> q = [q1, q2] (두 개의 해)
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

function J = calculate_jacobian(L, q)
    L1=L(1); L2=L(2); q1=q(1); q2=q(2); s1=sin(q1); c1=cos(q1); s12=sin(q1+q2); c12=cos(q1+q2);
    J = [-L1*s1-L2*s12, -L2*s12;
          L1*c1+L2*c12,  L2*c12];
end

function visualize_robot(L, q, target_pos, fig_title)
    figure('Name', fig_title);
    pos_j1 = forward_kinematics([L(1),0], q);
    pos_ee = forward_kinematics(L, q);
    
    plot([0, pos_j1(1), pos_ee(1)], [0, pos_j1(2), pos_ee(2)], 'o-', 'LineWidth', 3, 'MarkerSize', 8, 'MarkerFaceColor', 'k');
    hold on;
    if ~isempty(target_pos), plot(target_pos(1), target_pos(2), 'rx', 'MarkerSize', 15, 'LineWidth', 2); end
    grid on; axis equal; axis([-sum(L), sum(L), -sum(L), sum(L)]*1.2);
    title(fig_title); xlabel('X-axis (m)'); ylabel('Y-axis (m)');
end

function animate_trajectory(L, q_start, q_end, fig_title)
    % 동적 시각화
    figure('Name', fig_title);
    max_reach = sum(L);
    
    for t = linspace(0, 1, 100)
        q_current = q_start*(1-t) + q_end*t;
        pos_j1 = forward_kinematics([L(1),0], q_current);
        pos_ee = forward_kinematics(L, q_current);
        
        cla;
        plot([0, pos_j1(1), pos_ee(1)], [0, pos_j1(2), pos_ee(2)], 'o-', 'LineWidth', 3, 'MarkerSize', 8, 'MarkerFaceColor', 'k');
        grid on; axis equal; axis([-max_reach, max_reach, -max_reach, max_reach]*1.2);
        title(sprintf('%s (%.0f%%)', fig_title, t*100));
        drawnow;
    end
end