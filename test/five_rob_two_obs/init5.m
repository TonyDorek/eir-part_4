clear;
clc;

n_robots = 5;
thr = 5;
range = 100;
eps = 0.01; % Disconnection: eps = 0;

kx = 0.6 * ones(1, n_robots);
ky = 0.6 * ones(1, n_robots);

for i = 1:n_robots
    K{i} = [kx(i), 0;
             0, ky(i)];
end

for i = 1:n_robots
    eval(sprintf('K%d = K{%d};', i, i));
end

X0 = [0, 102, 50, 100, 0;
      0, 56, -20, 40, 20];

Xd = [100, 2, 20, 40, 200;
       60, 4, 40, 40, 100];

Xd_dot = zeros(2, n_robots);

for i = 1:n_robots
    eval(sprintf('X%dd = Xd(:,%d);', i, i));       % crea X1d, X2d, X3d, X4d, X5d
    eval(sprintf('X%dd_dot = Xd_dot(:,%d);', i, i)); % crea anche X1d_dot, ecc.
    eval(sprintf('X%02d = X0(:,%d);', i, i));       % opzionale: anche X01, X02, ecc.
end

obj1 = [60, 45];
obj2 = [30, 25];

figure;
set(gcf, 'Position', [250, 250, 1200, 500]); % [x, y, width, height]

subplot(1, 2, 1); % Create a 1x2 grid, access the 1st plot
create_graph(X0,range,'Initial robot positions (nodes) and connections (edges)');

subplot(1, 2, 2); % Access the 2nd plot
create_graph(Xd,range,'Desired robot positions (nodes) and connections (edges)');
