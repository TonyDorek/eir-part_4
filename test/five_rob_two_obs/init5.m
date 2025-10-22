%% === INPUT INITIALIZATION ===

clear;
clc;

n_robots = 5;

% --- Robot control: parameters ---
X0 = [0, 102, 50, 100, 0;
      0, 56, -20, 40, 20];

Xd = [100, 2, 20, 40, 200;
       60, 4, 40, 40, 100];

Xd_dot = zeros(2, n_robots);

for i = 1:n_robots
    eval(sprintf('X%dd = Xd(:,%d);', i, i));       % create X1d, X2d, X3d, X4d, X5d
    eval(sprintf('X%dd_dot = Xd_dot(:,%d);', i, i)); % create X1d_dot, ecc.
    eval(sprintf('X%02d = X0(:,%d);', i, i));       % create X01, X02, ecc.
end

kx = 0.6 * ones(1, n_robots);
ky = 0.6 * ones(1, n_robots);

for i = 1:n_robots
    K{i} = [kx(i), 0;
             0, ky(i)];
end

for i = 1:n_robots
    eval(sprintf('K%d = K{%d};', i, i));
end

% --- Collision avoidance: parameters ---
thr = 5; % Minimum distance threshold. Collision: thr = 0
obj1 = [60, 45]; % Fixed obstacle 1 position
obj2 = [30, 25]; % Fixed obstacle 2 position

% --- Connectivity maintenance: parameters ---
range = 100; % Maximum connectivity range
eps = 0.1; % Connection strength. Disconnection: eps = 0

% --- Graph plots ---
figure;
set(gcf, 'Position', [250, 250, 1200, 500]); % [x, y, width, height]

subplot(1, 2, 1); % Create a 1x2 grid, access the 1st plot
[~,p0] = create_graph(X0,range,'Initial robot positions (nodes) and connections (edges)');

subplot(1, 2, 2); % Access the 2nd plot
[~,pd] = create_graph(Xd,range,'Desired robot positions (nodes) and connections (edges)');
