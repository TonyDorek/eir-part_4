%% === INPUT INITIALIZATION ===

clear;
clc;

% --- Robots/Objects parameters ---
n_robots = 5;       % Actually fixed for now
n_obj = 4;          % How many obstacled?

% --- "Randomizations" ---
spawnrange = 24;    % How spreaded out we want our position generated?
random = false;      % if random == true, we randomize; if random != true, we use our fixed simulation with 5 robots

if random~=true
    n_obj=2;
end

% --- Barrier Enforcement Block parameters ---
cbf_states=n_robots*2;
cbf_actions=n_robots*2;
cbf_rob_obj = n_obj*n_robots;                   % each obj for each robot
cbf_rob_rob = ((n_robots*(n_robots-1))/2);      % robot each other
cbf_total_number = cbf_rob_obj + cbf_rob_rob + 1;  % 1 for Global Connectivity

% --- Collision avoidance: parameters ---
thr = 2;        % Minimum distance threshold. Collision: thr = 0

obstacles = zeros(2,n_obj);   % Initialize the obstacles array

if random~=true
    obstacles(:,1) = [60, 45];  % Fixed obstacle 1 position
    obstacles(:,2) = [30, 25];  % Fixed obstacle 2 position
else
    obstacles = randomobj(n_obj,spawnrange);
end

% --- Connectivity maintenance: parameters ---
range = 100; % Maximum connectivity range
eps_global = 0.1; % Global connection strength. Ineffectiveness: eps_global = 0
eps_local = 50; % Local connection strength. Ineffectiveness: eps_local >>> range (e.g. eps_local = 100*range)


% --- Robot control: parameters ---

% Initial positions
if random~=true
    X0 = [0, 102, 50, 100, 0;
          0, 56, -20, 40, 20]; 
else
    X0 = randompos(n_robots,thr,spawnrange);
end

% Desired positions
if random~=true
    Xd = [100, 2, 20, 40, 200;
           60, 4, 40, 40, 100];
else
    Xd = randompos(n_robots,thr,spawnrange);
end

Xd_dot = zeros(2, n_robots);

for i = 1:n_robots
    eval(sprintf('X%dd = Xd(:,%d);', i, i));            % create X1d, X2d, X3d, X4d, X5d
    eval(sprintf('X%dd_dot = Xd_dot(:,%d);', i, i));    % create X1d_dot, ecc.
    eval(sprintf('X%02d = X0(:,%d);', i, i));           % create X01, X02, ecc.
end

% Controller gains
kx = 0.6 * ones(1, n_robots);
ky = 0.6 * ones(1, n_robots);

for i = 1:n_robots
    K{i} = [kx(i), 0;
             0, ky(i)];
end

for i = 1:n_robots
    eval(sprintf('K%d = K{%d};', i, i));
end

% --- Graph plots ---
figure;
set(gcf, 'Position', [250, 250, 1200, 500]);    % [x, y, width, height]

subplot(1, 2, 1); % Create a 1x2 grid, access the 1st plot
[~,p0] = create_graph(X0, range, 'Initial robot positions (nodes) and connections (edges)');

subplot(1, 2, 2); % Access the 2nd plot
[~,pd] = create_graph(Xd, range, 'Desired robot positions (nodes) and connections (edges)');

