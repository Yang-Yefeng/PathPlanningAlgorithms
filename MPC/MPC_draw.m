function MPC_draw()

data = csvread('MPC_linear_track.csv', 1);

time = data(:, 1);

px = data(:, 2);
py = data(:, 3);
pz = data(:, 4);

vx = data(:, 5);
vy = data(:, 6);
vz = data(:, 7);

ax = data(:, 8);
ay = data(:, 9);
az = data(:, 10);

jx = data(:, 11);
jy = data(:, 12);
jz = data(:, 13);

px_ref = data(:, 14);
py_ref = data(:, 15);
pz_ref = data(:, 16);

figure(1);
plot3(px, py, pz, 'linewidth', 2); hold on;
plot3(px_ref, py_ref, pz_ref, 'linewidth', 2);
title('Linear MPC Tracking');
grid on;

figure(2);
plot(time, px, 'linewidth', 2); hold on;
plot(time, vx, 'linewidth', 2); hold on;
plot(time, ax, 'linewidth', 2); hold on;
plot(time, jx, 'linewidth', 2); hold on;
grid on;
ylabel('X');
xlabel('time');

figure(3);
plot(time, py, 'linewidth', 2); hold on;
plot(time, vy, 'linewidth', 2); hold on;
plot(time, ay, 'linewidth', 2); hold on;
plot(time, jy, 'linewidth', 2); hold on;
grid on;
ylabel('Y');
xlabel('time');

figure(4);
plot(time, pz, 'linewidth', 2); hold on;
plot(time, vz, 'linewidth', 2); hold on;
plot(time, az, 'linewidth', 2); hold on;
plot(time, jz, 'linewidth', 2); hold on;
grid on;
ylabel('Z');
xlabel('time');


end