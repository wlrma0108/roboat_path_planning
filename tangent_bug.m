clc;
clear;
close all;

start_point = [0, 0];
goal_point = [100, 100];

wall1 = [40 30; 60 30; 60 50; 40 50];
trap_wall_left = [70, 75; 75, 75; 75, 95; 70, 95]; 
trap_wall_back = [70, 95; 90, 95; 90, 100; 70, 100];
trap_wall_right = [90, 75; 95, 75; 95, 100; 90, 100];

all_the_walls = {wall1, trap_wall_left, trap_wall_back, trap_wall_right};

sensor_range = 40;
move_dist = 0.5;
my_pos = start_point;
path_history = my_pos;

all_corners = [];
for k=1:length(all_the_walls)
    all_corners = [all_corners; all_the_walls{k}];
end

direction_vec = (goal_point - my_pos) / norm(goal_point - my_pos); % 현재 진행방향을 저장할 변수

figure;
hold on;
axis equal;
grid on;
title('TangentBug Algorithm - Collision Fixed');
xlabel('x축');
ylabel('y축');

plot([start_point(1), goal_point(1)], [start_point(2), goal_point(2)], 'r--');
for k=1:length(all_the_walls)
    wall = all_the_walls{k};
    fill(wall(:,1), wall(:,2), [0.7 0.7 0.7]);
end

plot(start_point(1), start_point(2), 'go', 'MarkerFaceColor', 'g', 'MarkerSize', 10);
plot(goal_point(1), goal_point(2), 'rx', 'MarkerSize', 12', 'LineWidth', 2);
robot_drawing = plot(my_pos(1), my_pos(2), 'bo', 'MarkerFaceColor', 'b', 'MarkerSize', 8);
path_drawing = plot(path_history(:,1), path_history(:,2), 'b-', 'LineWidth', 1.5);
sensor_circle = plot(0,0, 'g--');

while(1)
    
    if norm(my_pos - goal_point) < move_dist
        disp('도착 성공!');
        break;
    end
    
    visible_points = [];
    
    dist_to_goal = norm(my_pos - goal_point);
    if dist_to_goal <= sensor_range
        path_is_clear = true;
        for i=1:10
            p = my_pos + (i/10)*(goal_point-my_pos);
            for k=1:length(all_the_walls), if inpolygon(p(1), p(2), all_the_walls{k}(:,1), all_the_walls{k}(:,2)), path_is_clear = false; break; end, end
            if ~path_is_clear, break; end
        end
        if path_is_clear, visible_points = [visible_points; goal_point]; end
    end
    
    for i = 1:size(all_corners, 1)
        corner = all_corners(i, :);
        dist_to_corner = norm(my_pos - corner);
        if dist_to_corner <= sensor_range
            path_is_clear = true;
            for j=1:10
                p = my_pos + (j/10)*(corner-my_pos);
                for k=1:length(all_the_walls), if inpolygon(p(1), p(2), all_the_walls{k}(:,1), all_the_walls{k}(:,2)), path_is_clear = false; break; end, end
                if ~path_is_clear, break; end
            end
            if path_is_clear, visible_points = [visible_points; corner]; end
        end
    end
    
    best_target = [];
    min_heuristic_dist = inf;
    
    for i = 1:size(visible_points, 1)
        target_point = visible_points(i, :);
        heuristic_dist = norm(my_pos - target_point) + norm(target_point - goal_point);
        if heuristic_dist < min_heuristic_dist
            min_heuristic_dist = heuristic_dist;
            best_target = target_point;
        end
    end

    ideal_direction = (goal_point - my_pos) / norm(goal_point - my_pos);
    if ~isempty(best_target)
        ideal_direction = (best_target - my_pos) / norm(best_target - my_pos);
    end

    % ▼▼▼ 핵심 수정 부분: 이동 전 충돌 확인 로직 ▼▼▼
    next_pos_candidate = my_pos + move_dist * ideal_direction;
    is_colliding = false;
    for k=1:length(all_the_walls)
        if inpolygon(next_pos_candidate(1), next_pos_candidate(2), all_the_walls{k}(:,1), all_the_walls{k}(:,2))
            is_colliding = true;
            break;
        end
    end
    
    if ~is_colliding
        % 충돌하지 않으면 계획대로 이동
        my_pos = next_pos_candidate;
        direction_vec = ideal_direction;
    else
        % 충돌하면, 벽을 따라 미끄러지는 움직임 수행
        dir_x = direction_vec(1);
        dir_y = direction_vec(2);
        right_dir = [dir_y, -dir_x];
        straight_dir = direction_vec;
        left_dir = [-dir_y, dir_x];
        
        directions_to_try = {right_dir, straight_dir, left_dir};
        
        for i=1:3
            next_dir = directions_to_try{i};
            next_pos = my_pos + move_dist * next_dir;
            
            is_stuck = false;
            for k=1:length(all_the_walls)
                if inpolygon(next_pos(1), next_pos(2), all_the_walls{k}(:,1), all_the_walls{k}(:,2))
                    is_stuck = true;
                    break;
                end
            end
            
            if ~is_stuck
                my_pos = next_pos;
                direction_vec = next_dir;
                break;
            end
        end
    end
    % ▲▲▲ 핵심 수정 부분 ▲▲▲

    path_history = [path_history; my_pos];
    set(robot_drawing, 'XData', my_pos(1), 'YData', my_pos(2));
    set(path_drawing, 'XData', path_history(:,1), 'YData', path_history(:,2));
    
    t = linspace(0,2*pi,100);
    set(sensor_circle, 'XData', my_pos(1) + sensor_range*cos(t), 'YData', my_pos(2) + sensor_range*sin(t));
    
    pause(0.001);
end