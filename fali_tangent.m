%% Tangent Bug Algorithm - Complete Version
clear; clc; close all;

%% 0. 환경 및 로봇 설정
% 맵 생성
mapData = zeros(50, 50);
mapData(15:35, 20:23) = 1; % 장애물 1
mapData(15:18, 35:40) = 1; % 장애물 2 (U자형)
mapData(32:35, 35:40) = 1;
mapData(15:35, 40:42) = 1;
map = binaryOccupancyMap(mapData);

% 시작점, 목표점, 로봇 파라미터
q_init = [5, 25];
q_goal = [45, 25];
robot_pos = q_init;
R = 8; 
step_size = 1;
path = [robot_pos];

% 알고리즘 상태 변수
state = "motion-to-goal";
d_heuristic_prev = inf;
d_min = inf; % d_min 초기화
boundary_points_memory = []; % d_min 계산을 위한 메모리

% 시각화
figure('Name', 'Tangent Bug Algorithm - Complete');
show(map); hold on;
plot(q_init(1), q_init(2), 'ro', 'MarkerFaceColor', 'r');
plot(q_goal(1), q_goal(2), 'go', 'MarkerFaceColor', 'g');
legend('Start', 'Goal', 'Location', 'northeastoutside');

%% 1. 메인 루프
for iter = 1:2000
    if norm(robot_pos - q_goal) < step_size, disp('Goal Reached!'); break; end

    sensed_points = getSensorData(robot_pos, R, map);
    Oi_star = [];
    title_str = sprintf('State: %s', state);

    switch state
        case "motion-to-goal"
            if isempty(sensed_points)
                direction = (q_goal - robot_pos) / norm(q_goal - robot_pos);
                d_heuristic_prev = norm(robot_pos - q_goal);
            else
                Oi_points = sensed_points;
                d_heuristic_all = sqrt(sum((Oi_points - robot_pos).^2, 2)) + sqrt(sum((Oi_points - q_goal).^2, 2));
                [d_h_current, min_idx] = min(d_heuristic_all);
                Oi_star = Oi_points(min_idx, :);
                
                if d_h_current > d_heuristic_prev
                    state = "boundary-following";
                    disp('Switching to -> boundary-following');
                    % d_min과 메모리 초기화
                    d_min = min(sqrt(sum((Oi_points - q_goal).^2, 2)));
                    boundary_points_memory = Oi_points;
                    % 가장 가까운 장애물 지점으로 초기 방향 설정
                    [~, closest_idx] = min(sqrt(sum((Oi_points - robot_pos).^2, 2)));
                    direction = (Oi_points(closest_idx, :) - robot_pos) / norm(Oi_points(closest_idx, :) - robot_pos);
                else
                    direction = (Oi_star - robot_pos) / norm(Oi_star - robot_pos);
                    d_heuristic_prev = d_h_current;
                end
            end
        
        case "boundary-following"
            % ==========================================================
            % ## 로직 7: 값들의 지속적인 업데이트
            % ==========================================================
            if isempty(sensed_points)
                % 장애물을 놓치면 다시 motion-to-goal 상태로
                state = "motion-to-goal";
                direction = (q_goal - robot_pos) / norm(q_goal - robot_pos);
            else
                % d_leave 계산: 현재 보이는 장애물 지점들 중 목표와 가장 가까운 거리
                dist_to_goal = sqrt(sum((sensed_points - q_goal).^2, 2));
                [d_leave, leave_idx] = min(dist_to_goal);
                
                % d_min 업데이트: 지금까지 본 모든 장애물 지점들 중 목표와 가장 가까운 거리
                boundary_points_memory = [boundary_points_memory; sensed_points];
                d_min = min(d_min, d_leave);

                title_str = sprintf('State: %s | d_leave: %.1f, d_min: %.1f', state, d_leave, d_min);

                % ==========================================================
                % ## 로직 8: '목표 지향 이동'으로 복귀 조건
                % ==========================================================
                if d_leave < d_min
                    state = "motion-to-goal";
                    disp('Switching to -> motion-to-goal (Found better path)');
                    % d_heuristic 초기화하여 즉시 상태 전환이 일어나지 않도록 함
                    d_heuristic_prev = inf; 
                    % 더 나은 경로(d_leave를 만족시킨 점)를 향해 방향 설정
                    direction = (sensed_points(leave_idx, :) - robot_pos) / norm(sensed_points(leave_idx, :) - robot_pos);
                else
                    % 조건 만족 못하면 계속 경계면 따라 이동
                    % 가장 가까운 벽 지점을 찾아 벽을 왼쪽에 두고 이동하는 방향 계산
                    [~, closest_idx] = min(sqrt(sum((sensed_points - robot_pos).^2, 2)));
                    wall_vec = sensed_points(closest_idx, :) - robot_pos;
                    % wall_vec를 반시계 방향으로 90도 회전시켜 벽에 접하는 방향 계산
                    direction = [-wall_vec(2), wall_vec(1)];
                    direction = direction / norm(direction);
                end
            end
    end

    % 로봇 이동 로직 (충돌 방지)
    proposed_pos = robot_pos + direction * step_size;
    if checkOccupancy(map, proposed_pos) == 0
        robot_pos = proposed_pos;
    else 
        % 만약 막혔다면, 제자리에서 약간 회전하여 길을 찾음
        found_path = false;
        for angle_offset = linspace(0, pi, 10)
            angle = atan2(direction(2), direction(1)) + angle_offset;
            test_dir = [cos(angle), sin(angle)];
            if checkOccupancy(map, robot_pos + test_dir*step_size) == 0
                direction = test_dir;
                robot_pos = robot_pos + direction * step_size;
                found_path = true;
                break;
            end
        end
        if ~found_path, direction = -direction; end % 정 갈 곳이 없으면 반대 방향으로
    end
    path = [path; robot_pos];

    % 시각화 업데이트
    cla; show(map);
    plot(q_init(1), q_init(2), 'ro', 'MarkerFaceColor', 'r');
    plot(q_goal(1), q_goal(2), 'go', 'MarkerFaceColor', 'g');
    plot(path(:,1), path(:,2), 'b-', 'LineWidth', 1.5);
    plot(robot_pos(1), robot_pos(2), 'ko', 'MarkerFaceColor', 'k');
    viscircles(robot_pos, R, 'Color', 'r', 'LineStyle', '--');
    if ~isempty(sensed_points), plot(sensed_points(:,1), sensed_points(:,2), 'm.'); end
    if ~isempty(Oi_star), plot(Oi_star(1), Oi_star(2), 'yx', 'MarkerSize', 12, 'LineWidth', 2); end
    title(sprintf('%s | Iter: %d', title_str, iter));
    drawnow;
end
if iter == 2000, disp('Max iterations reached.'); end

%% 2. 로컬 함수
function [points] = getSensorData(pos, R, map)
    points = [];
    angles = linspace(0, 2*pi, 100);
    for i = 1:length(angles)
        for r = 1:R
            check_pos = pos + r * [cos(angles(i)), sin(angles(i))];
            if checkOccupancy(map, check_pos) == 1, points = [points; check_pos]; break; end
        end
    end
end