function trajectory = Bezier_Curve(pts, visualization)
%% Pts:     2-D points with the formation of Pts = [pt1; pt2; ...; ptn]
n_point = length(pts);
t = 0 : 0.01 : 1;
trajectory_x = 0;
trajectory_y = 0;
for i = 0 : n_point - 1
    coefficient = nchoosek(n_point - 1, i) .* t.^i .* (1 - t).^(n_point - 1 - i);
    trajectory_x = trajectory_x + coefficient * pts(i + 1, 1);
    trajectory_y = trajectory_y + coefficient * pts(i + 1, 2);
end
trajectory = [trajectory_x; trajectory_y];
%% plotting
if visualization
    figure(1)
    for iter2  = 1 : 1 : n_point
        plot(pts(iter2, 1), pts(iter2, 2), 'ro');
        hold on;
    end
    plot(trajectory_x, trajectory_y);
    hold off;
end
end