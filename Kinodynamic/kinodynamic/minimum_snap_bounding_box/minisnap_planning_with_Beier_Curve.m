function trajectory_coe = minisnap_planning_with_Beier_Curve(flag, pts, bounding_box, time, order, v_max, a_max, j_max)
%% comment
% flag:             1: x,  2: y
% pts:              waypoints
% bounding_box:     bounding_box
% time:             time
% order:            order
% v_max:            maximum velocity
% a_max:            maximum acceleration
% j_max:            maximum jerk

%% boundary condition and equality constraints
% initial condition is the p, v, a, j of the robot at way point(1)
% terminal condition is the p, v, a, j of the robot at way point(end)
n_pts = size(pts, 1);
initial_condition = [pts(1), 0, 0, 0];
terminal_condition = [pts(end), 0, 0, 0];
[Q, M]  = calculate_Q_M(n_pts, order, time);
Q = M'*Q*M;
Q = nearestSPD(Q);
[Aeq, beq] = get_Abeq(n_pts, order, time, initial_condition, terminal_condition);

%% Inequality constraints
bounding_box_range = zeros(n_pts, 2);
disp(n_pts)
for k = 1 : n_pts
    bounding_box_range(k , :) = [bounding_box(flag, k) - bounding_box(flag + 2, k), bounding_box(flag, k) + bounding_box(flag + 2, k)];
end
[Aieq, bieq] = getAbieq(n_pts, order, bounding_box_range, time, v_max, a_max, j_max);
f = zeros(size(Q, 1), 1);
trajectory_coe = quadprog(Q, f, Aieq, bieq, Aeq, beq);
end

function [Aieq, bieq] = getAbieq(n_pts, order, bounding_box_range, time, v_max, a_max, j_max)
n_all_poly = n_pts * (order + 1);
%% position constraints
coeff_p = ones(n_all_poly, 1);
bieq_p = [];
for k = 1 : n_pts % max
    coeff_p(1 + (k - 1) * (order + 1) : k * (order  +1)) = coeff_p(1 + (k-1)*(order+1):k*(order+1)) * time(k)^(1);
    bieq_p = [bieq_p ; ones(order + 1, 1) * bounding_box_range(k, 2)];
end
for k = 1 : n_pts % -min
    bieq_p = [bieq_p ; ones(order + 1, 1) * bounding_box_range(k, 1) * (-1)];
end
Aieq_p = diag(coeff_p, 0);
Aieq_p = [Aieq_p ; -Aieq_p];

%% velocity constraints
n_ctr = order;              % the number of control posints after first deferention: n_order 
n_eq = order * n_pts * 2;   % number of equations (max and min constraints)
Aieq_v = zeros(n_eq / 2 , n_all_poly);

for k = 1 : n_pts
    for n = 1 : n_ctr
        index_col = k * (order + 1) - 1;
        index_row = n + (k - 1) * n_ctr;
        Aieq_v(index_row, index_col : index_col + 1) = order * [-1, 1] * time(k)^(0);
    end
end
Aieq_v = [Aieq_v; -Aieq_v];
bieq_v = ones(n_eq, 1)* v_max;

%% acceleration constraints
n_ctr = order - 1;    % the number of control posints after second deferention: n_order - 1 
n_eq = n_ctr * n_pts * 2; % number of equations (max and min constraints)
Aieq_a = zeros(n_eq / 2, n_all_poly);

for k = 1:n_pts
    for n = 1:n_ctr
        index_col = k*(order+1)-2;
        index_row = n+(k-1)*n_ctr;
        Aieq_a(index_row,index_col:index_col+2) = order * (order-1) * [1, -2, 1] * time(k)^(-1);
    end
end
Aieq_a = [Aieq_a;-Aieq_a];
bieq_a = ones(n_eq,1)*a_max;

%% jerk constraints
n_ctr = order - 2;    % the number of control posints after third deferention: n_order - 2
n_eq = n_ctr * n_pts * 2; % number of equations (max and min constraints)
Aieq_j = zeros(n_eq / 2, n_all_poly);

for k = 1 : n_pts
    for n = 1 : n_ctr
        index_col = k * (order + 1) - 3;
        index_row = n + (k - 1) * n_ctr;
        Aieq_j(index_row, index_col : index_col + 3) = order * (order - 1) * (order - 2) * [1, -3, 3, -1] * time(k)^(-2);
    end
end
Aieq_j = [Aieq_j ; -Aieq_j];
bieq_j = ones(n_eq, 1) * j_max;

%% combine them together
Aieq = [Aieq_p; Aieq_v; Aieq_a; Aieq_j];
bieq = [bieq_p; bieq_v; bieq_a; bieq_j];
end

function [Aeq, beq] = get_Abeq(n_pts, order, time, init_cond, end_cond)
n_all_poly = n_pts * (order + 1);
%% constrints for initial condition
Aeq_start = zeros(4, n_all_poly);   % Ascending order
Aeq_start(1, 1) = 1 * time(1)^(1);    % c0
Aeq_start(2, 1: 2) = order * [-1,1] * time(1)^(0);    % c'0 = n*(-c0 + c0)
Aeq_start(3, 1: 3) = order * (order-1) * [1, -2, 1] * time(1)^(-1); % c''0 = n*(n-1)*(c2 -2*c1 +c0)   
Aeq_start(4, 1: 4) = order * (order-1) * (order-2) * [1, -3, 3, -1] * time(1)^(-2);   % c'''0 = n*(n-1)* (n-2) *(c3 - 3*c2 + 3*c1 - c0)   
beq_start = init_cond';

%% constrints for terminal condition
Aeq_end = zeros(4, n_all_poly); % Descending order
Aeq_end(1,end) = 1 * time(end)^(1);% cn
Aeq_end(2,end-1:end) = order * [-1, 1] * time(end)^(0);% c'n-1 = n*(cn -cn-1)
Aeq_end(3,end-2:end) = order * (order-1) * [1, -2, 1]  * time(end)^(-1);% c''n-2 = n^2*(n-1)*(cn - 2*cn-1 + cn-2)    
Aeq_end(4,end-3:end) = order * (order-1) * (order-2) * [1, -3, 3, -1] * time(end)^(-2);% c''n-3 = n*(n-1)*(n-2)*(cn - 3*cn-1 + 3*cn-2 - c0)   
beq_end = end_cond';

%% position should be identical between two segments
Aeq_con_p = zeros(n_pts - 1, n_all_poly);
for k = 1 : n_pts - 1
    Aeq_con_p(k, k * (order + 1)) = 1 * time(k)^(1);
    Aeq_con_p(k, k * (order + 1) + 1) = -1 * time(k + 1)^(1);
end
beq_con_p = zeros(n_pts - 1,1);

%% velocity shoule be identical between two segments
Aeq_con_v =  zeros(n_pts - 1, n_all_poly);
for k = 1 : n_pts - 1 % (c(n))- c(n-1)) segment 1 + (-c1 + c0) segment 2
    Aeq_con_v(k, k * (order + 1) - 1 : k * (order + 1)) = [-1, 1] * time(k)^(0);
    Aeq_con_v(k, k * (order + 1) + 1 : k * (order + 1) + 2) = [1, -1] * time(k + 1)^(0);
end    
beq_con_v = zeros(n_pts - 1,1);

%% acceleration shoule be identical between two segments
Aeq_con_a = zeros(n_pts - 1, n_all_poly);
for k = 1 : n_pts - 1 % (c(n))- 2*c(n-1) + c(n-2)) segment 1 + (-c2 + 2*c1 - c0) segment 2
    Aeq_con_a(k, k * (order + 1) - 2 : k * (order + 1)) = [1, -2, 1] * time(k)^(-1);
    Aeq_con_a(k, k * (order + 1) + 1 : k * (order + 1) + 3) = [-1, 2, -1] * time(k + 1)^(-1);
end  
beq_con_a = zeros(n_pts - 1,1);

%% jerk shoule be identical between two segments
Aeq_con_j = zeros(n_pts - 1, n_all_poly);
for k = 1 : n_pts - 1 % (c(n))- 3*c(n-1) + 3*c(n-2) - c(n-3)) segment 1 + (-c3 + 3*c2 - 3*c1 + c0) segment 2
    Aeq_con_j(k, k * (order + 1) - 3 : k * (order + 1)) = [1, -3, 3, -1] * time(k)^(-2);
    Aeq_con_j(k, k * (order + 1) + 1 : k * (order + 1) + 4) = [-1, 3, -3, 1] * time(k+1)^(-2);
end  
beq_con_j = zeros(n_pts-1,1);

%% combine all constraints together
Aeq_con = [Aeq_con_p; Aeq_con_v; Aeq_con_a; Aeq_con_j];
beq_con = [beq_con_p; beq_con_v; beq_con_a; beq_con_j];
Aeq = [Aeq_start; Aeq_end; Aeq_con];
beq = [beq_start; beq_end; beq_con];
end

function [Q, M] = calculate_Q_M(n_pts, order, time)
Q = [];
M = [];
M_k = coefficient_transform_matrix(order);
for k = 1 : n_pts
    Q_k = zeros(order + 1, order + 1);
    coe = @(x) x*(x-1)*(x-2)*(x-3);
    for i = 0 : order
        for j = 0 : order
            if (i < 4) || (j < 4)
                continue;       % should be zero after 4-th differential
            else
                Q_k(i + 1,j + 1) = coe(i) * coe(j)/(i + j - 7) * time(k)^(j + i - 7); 
            end
        end
    end
    Q = blkdiag(Q, Q_k);        % a huge diag matrix which elements are small matrixes
    M_k = M_k * (eye(order + 1)) .*time(k);
    M = blkdiag(M, M_k);        % a huge diag matrix which elements are small matrixes
end
end

function M = coefficient_transform_matrix(order)
if order == 3
    M = [ 1   0   0  0;
          -3   3   0  0;
           3  -6   3  0;
          -1   3  -3  1];
elseif order == 4       % Degree D = 4
    M = [1   0   0   0  0 ;
         -4   4   0   0  0 ;
          6 -12   6   0  0 ;
         -4  12 -12   4  0 ;
          1  -4   6  -4  1 ];
elseif order == 5  % Degree D = 5
    M = [1   0   0   0  0  0
         -5   5   0   0  0  0;
         10 -20  10   0  0  0;
        -10  30 -30  10  0  0;
         5  -20  30 -20  5  0;
        -1    5 -10  10 -5  1 ];    
elseif order == 6  % Degree D = 6
    M = [1    0    0    0    0    0    0;
         -6    6    0    0    0    0    0;
         15  -30   15    0    0    0    0; 
        -20   60  -60   20    0    0    0;
         15  -60   90  -60   15    0    0;
         -6   30  -60   60  -30    6    0;
          1   -6   15  -20   15   -6    1 ];
elseif order == 7 % Degree D = 7
    M = [ 1    0    0    0    0   0  0  0;
          -7    7    0    0    0   0  0  0;
          21  -42   21    0    0   0  0  0
         -35  105 -105   35    0   0  0  0;
          35 -140  210 -140   35   0  0  0;
         -21  105 -210  210 -105  21  0  0;
           7  -42  105 -140  105 -42  7  0;
          -1   7   -21   35  -35  21 -7  1];
else
    fprintf('Unknown order...');
    M = [];
end
end