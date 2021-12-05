function main()
%% Configuration
clear;
clc;
%% points 
% pts = [[0 0];
%        [3 3];
%        [5 7];
%        [8 10]; 
%        [12 8];
%        [14 5];
%        [11 1];
%        [13, -1];
%        [11, -1];
%        [9, 2]];
% pts = [[2 1];
%        [6 5];
%        [10 9];
%        [6 12]];
pts = [[1 1];
       [5 1];
       [7 1 + 2*sqrt(3)];
       [5 1 + 4*sqrt(3)]; 
       [1 1 + 4*sqrt(3)];
       [-1 1 + 2*sqrt(3)];
       [1 1]];
%% bounding box
sizex = 5;
sizey = 5;
poly_order = 5;
v_max = 10;
a_max = 10;
j_max = 10;
num_points = size(pts, 1);
bounding_box = zeros(4, num_points);
for i = 1 : num_points
    bounding_box(:, i) = [pts(i, 1), pts(i, 2), sizex/2.0, sizey/2.0]';
    % center of the box, sizex, sizey
end

%% time allocation
time = zeros(1, num_points);
for i = 1 : num_points
    time(i) = 1;
end

%% minisnap_planning_with_Beier_Curve; by the way, x and y and calculated seperately.
trajectory_coex = minisnap_planning_with_Beier_Curve(1, pts(:, 1), bounding_box, time, poly_order, v_max, a_max, j_max);
disp('x finished');
trajectory_coey = minisnap_planning_with_Beier_Curve(2, pts(:, 2), bounding_box, time, poly_order, v_max, a_max, j_max);
disp('y finished');
% disp(size(trajectory_coex));
%% plotting
t = 0 : 0.01 : 1;
cx = zeros(num_points, poly_order + 1);
cy = zeros(num_points, poly_order + 1);
trajectory_x = zeros(1, length(t) * num_points);
trajectory_y = zeros(1, length(t) * num_points);
for i = 1 : num_points
    cx(i , :) = trajectory_coex(6 * i - 5 : 6 * i);     % one row
    cy(i , :) = trajectory_coey(6 * i - 5 : 6 * i);     % one row
    res = Bezier_Curve([cx(i , :); cy(i , :)]', false);
    trajectory_x((i - 1) * length(t) + 1 : i * length(t)) = res(1, :);
    trajectory_y((i - 1) * length(t) + 1 : i * length(t)) = res(2, :);
end
%% plotting for Bezier Curve with bounding box
% [trajectory_x, trajectory_y] = Bezier_Curve(pts, false);
figure(1)
% str = [repmat('(', length(pts(:, 1)), 1) num2str(pts(:, 1)) ...
%        repmat(',', length(pts(:, 2)), 1) num2str(pts(:, 2)) ...
%        repmat(')', length(pts(:, 2)), 1)];
% % way points with text
% plot(pts(:, 1), pts(:, 2), '-ro', 'MarkerFaceColor','r');
% text(pts(:, 1), pts(:, 2), cellstr(str));
% hold on;
for i = 1 : num_points
    rectangle = [[pts(i, 1) - sizex / 2, pts(i, 2) - sizey / 2];
                 [pts(i, 1) + sizex / 2, pts(i, 2) - sizey / 2];
                 [pts(i, 1) + sizex / 2, pts(i, 2) + sizey / 2];
                 [pts(i, 1) - sizex / 2, pts(i, 2) + sizey / 2];
                 [pts(i, 1) - sizex / 2, pts(i, 2) - sizey / 2]];
    for j = 1 : 4
        plot(rectangle(:, 1), rectangle(:, 2), 'g');
        hold on;
    end
end
% connect node
for i = 1 : num_points
    plot(cx(i, 1), cy(i, 1), 'ro', 'MarkerFaceColor','r'); hold on;
    str = [repmat('(', 1, 1) num2str(cx(i, 1)) repmat(',', 1, 1) num2str(cy(i, 1)) repmat(')', 1, 1)];
    text(cx(i, 1), cy(i, 1), cellstr(str)); hold on;
    
    plot(cx(i, poly_order + 1), cy(i, poly_order + 1), 'ro', 'MarkerFaceColor','r'); hold on;
    str = [repmat('(', 1, 1) num2str(cx(i, poly_order + 1)) repmat(',', 1, 1) num2str(cy(i, poly_order + 1)) repmat(')', 1, 1)];
    text(cx(i, poly_order + 1), cy(i, poly_order + 1), cellstr(str)); hold on;
end
% trajectory
plot(trajectory_x, trajectory_y, 'b', 'linewidth', 2);
hold off;
end