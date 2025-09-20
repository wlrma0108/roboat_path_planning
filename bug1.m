clc;
clear;
close all;

start_point = [0, 0];
goal_point = [100, 100];
the_wall = [50 20; 80 20; 80 60; 50 60];

move_dist = 0.5;
my_pos = start_point;
path_history = my_pos;

mode = 'GO';
direction_vec = (goal_point - my_pos) / norm(goal_point - my_pos);

hit_point = [];
leave_point = [];
min_dist_so_far = inf;
circle_step_counter = 0; % 추가된 변수

figure;
hold on;
axis equal;
grid on;
title('버그1 알고리즘 시뮬레이션 2020253091_김호중');
xlabel('x축');
ylabel('y축');

plot([start_point(1), goal_point(1)], [start_point(2), goal_point(2)], 'r--');
fill(the_wall(:,1), the_wall(:,2), [0.7 0.7 0.7]);
plot(start_point(1), start_point(2), 'go', 'MarkerFaceColor', 'g', 'MarkerSize', 10);
plot(goal_point(1), goal_point(2), 'rx', 'MarkerSize', 12', 'LineWidth', 2);
robot_drawing = plot(my_pos(1), my_pos(2), 'bo', 'MarkerFaceColor', 'b', 'MarkerSize', 8);
path_drawing = plot(path_history(:,1), path_history(:,2), 'b-', 'LineWidth', 1.5);

while(1)
    
    distance = norm(my_pos - goal_point);
    if distance < move_dist
        disp('도착 성공!');
        break;
    end
    
    if strcmp(mode, 'GO')
        direction_vec = (goal_point - my_pos) / norm(goal_point - my_pos);
        next_pos = my_pos + move_dist * direction_vec;
        if inpolygon(next_pos(1), next_pos(2), the_wall(:,1), the_wall(:,2))
            mode = 'CIRCLE';
            hit_point = my_pos;
            leave_point = my_pos;
            min_dist_so_far = norm(my_pos - goal_point);
            circle_step_counter = 0; % <-- 수정된 부분: 카운터 초기화
            
            temp = direction_vec(1);
            direction_vec(1) = -direction_vec(2);
            direction_vec(2) = temp;
        else
            my_pos = next_pos;
        end
        
    elseif strcmp(mode, 'CIRCLE')
        circle_step_counter = circle_step_counter + 1; 
        
        if norm(my_pos - hit_point) < move_dist && circle_step_counter > 5
             mode = 'RETURN';
             continue;
        end
        
        dist_to_goal = norm(my_pos - goal_point);
        if dist_to_goal < min_dist_so_far
            min_dist_so_far = dist_to_goal;
            leave_point = my_pos;
        end
        
        dir_x = direction_vec(1);
        dir_y = direction_vec(2);
        right_dir = [dir_y, -dir_x];
        next_pos_candidate = my_pos + move_dist * right_dir;
        
        if ~inpolygon(next_pos_candidate(1), next_pos_candidate(2), the_wall(:,1), the_wall(:,2))
            my_pos = next_pos_candidate;
            direction_vec = right_dir;
        else
            straight_dir = direction_vec;
            next_pos_candidate = my_pos + move_dist * straight_dir;
             if ~inpolygon(next_pos_candidate(1), next_pos_candidate(2), the_wall(:,1), the_wall(:,2))
                my_pos = next_pos_candidate;
                direction_vec = straight_dir;
             else
                left_dir = [-dir_y, dir_x];
                my_pos = my_pos + move_dist * left_dir;
                direction_vec = left_dir;
             end
        end
        
    elseif strcmp(mode, 'RETURN')
        if norm(my_pos - leave_point) < move_dist
            mode = 'GO';
            continue;
        end
        
        dir_x = direction_vec(1);
        dir_y = direction_vec(2);
        right_dir = [dir_y, -dir_x];
        next_pos_candidate = my_pos + move_dist * right_dir;
        
        if ~inpolygon(next_pos_candidate(1), next_pos_candidate(2), the_wall(:,1), the_wall(:,2))
            my_pos = next_pos_candidate;
            direction_vec = right_dir;
        else
            straight_dir = direction_vec;
            next_pos_candidate = my_pos + move_dist * straight_dir;
             if ~inpolygon(next_pos_candidate(1), next_pos_candidate(2), the_wall(:,1), the_wall(:,2))
                my_pos = next_pos_candidate;
                direction_vec = straight_dir;
             else
                left_dir = [-dir_y, dir_x];
                my_pos = my_pos + move_dist * left_dir;
                direction_vec = left_dir;
             end
        end
    end
    
    path_history = [path_history; my_pos];
    set(robot_drawing, 'XData', my_pos(1), 'YData', my_pos(2));
    set(path_drawing, 'XData', path_history(:,1), 'YData', path_history(:,2));
    
    pause(0.001);
end