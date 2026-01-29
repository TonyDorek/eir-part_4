%% INITIALIZING PARAMETERS
% Run it BEFORE executing the Simulink model...

clear;
clc;
close all;

s = rng(0);             % Seed for the random number generation
%s = rng('shuffle');

%% --- Starting configuration ---
CONFIG = struct();
CONFIG.randomize_robots_initpos = true;  % If true, robot initial positions are randomized
CONFIG.randomize_goals = true;           % If true, robot goal positions are randomized
CONFIG.randomize_obstacles = true;       % If true, obstacle positions and sizes are randomized
CONFIG.outlier_random_goal = false;       % If true, one random robot will have a random goal position
CONFIG.outlier_specific_goal = false;     % If true, one specific robot will have a specific goal position

%% --- Relative path (collector of functions) ---
addpath(genpath('./helpers'))

%% --- Simulation parameters ---    
N    = 5;                   % Number of robots
dim  = 2;                   % Space dimension
t_start = 0;                % Starting time
dt   = 0.05;                % Timestep
Tsim = 10;                  % Tot. simulation time
steps = floor(Tsim/dt);     % Tot. simulation 
Tstop = Tsim;               % The Simulink model uses Tstop as Stop Time
colors = hsv(N);            % Colours for the visualization script

%% --- Initial positions / velocities ---
R0 = 10;                                % Radius of the circle for the initial positions
theta = linspace(0,2*pi*(1-1/N),N)';    % Orientation of the initial positions
v = zeros(N,2);                         % Initial velocity vector
if CONFIG.randomize_robots_initpos
    x = [R0*cos(theta), R0*sin(theta)] + 0.5*randn(N,2);   % Initial position vector (circular distribution + random component)
else
    x = [R0*cos(theta), R0*sin(theta)];                    % Initial position vector (circular distribution)
end
x_start = x;

%% --- Controller parameters  ---
x0_vec = reshape(x', [], 1);   % Adapting init position in vector form for Simulink, 2N x 1
v0_vec = reshape(v', [], 1);   % Adapting init velocities in vector form for Simulink, 2N x 1
k_p = 1.0;                     % Position gain
k_d = 2.0;                     % Velocity gain

%% --- Control Barrier Function parameters ---
R_glob = 5.0;           % Global communication radius (for neighborhood determination)
R_loc = 4.0;            % Maximum local communication radius (for connectivity constraint)
conn_margin = 1.0;      % Margin for connectivity constraint. Enforce it only for pairs within this margin over R_loc                               
dmin = 1.0;             % Minimum distance between robots (for agent collision constraint)
rsafe = 0.25;           % Obstacle safety margin (for obstacle collision constraint, together with the obstacle radius)                                 
cbf_gain_conn = 1.0;    % Alpha multiplier for connectivity (alpha * h)
cbf_gain_col  = 5.0;    % Idem for agent collision
cbf_gain_obs  = 4.0;    % Idem for obstacle collision
Tpred = 0.5;            % Prediction horizon (seconds). Smaller = more conservative, larger = more predictive

%% -- Goal Setting -- 
if CONFIG.randomize_goals
    goal_region_center = [0 -1];
    goal_dispersion = 4.0;              % Controls spread of goals (higher = more dispersed)
    x_goal = goal_region_center + goal_dispersion * (2*rand(N,2) - 1);      % N×2 matrix of individual goals
    outlier_idx = -1;                   % Initialize outlier index
    if CONFIG.outlier_random_goal
        outlier_idx = randi(N);         % Random robot gets outlier goal
        outlier_direction = randn(1,2);
        outlier_direction = outlier_direction / norm(outlier_direction);
        outlier_distance = 5.0;         % Additional distance for outlier goal
        x_goal(outlier_idx,:) = goal_region_center + (goal_dispersion + outlier_distance) * outlier_direction;
        fprintf('Random goal assigned to random outlier Robot %d\n', outlier_idx); % Debug info
    end
else
    x_goal = ones(N,1)*[0 -1];  % Fixed goal for all the robots
    if CONFIG.outlier_specific_goal
        outlier_idx = 2; 
        x_goal(outlier_idx,:) = [0 -6];
        fprintf('Specific goal assigned to specific outlier Robot %d\n', outlier_idx);
    end
end

%% --- Obstacles ---
if CONFIG.randomize_obstacles
    nObs = 4;                            % Number of obstacles
    obs_pos = 8 * (2*rand(nObs,2) - 1);  % Randomized obstacle positions within workspace. Random positions in [-8,8]×[-8,8]
    obs_rad = 0.3 + 0.3*rand(nObs,1);    % Random radii between 0.3 and 0.6

    if CONFIG.randomize_robots_initpos   % Validate and regenerate goals if too close to obstacles
        goal_obstacle_clearance = 1.0;   % Minimum distance from obstacle edge to goal
        max_attempts = 100;

        for i = 1:N
            attempts = 0;
            valid = false;
            
            while ~valid && attempts < max_attempts
                valid = true;
                for o = 1:nObs     % Check distance to all obstacles
                    dist_to_obs = norm(x_goal(i,:) - obs_pos(o,:)) - obs_rad(o);
                    if dist_to_obs < goal_obstacle_clearance
                        valid = false;
                        break;
                    end
                end
                
                
                if ~valid          % If invalid, regenerate this goal
                    if i == outlier_idx && CONFIG.outlier_random_goal   % Regenerate outlier
                        outlier_direction = randn(1,2);
                        outlier_direction = outlier_direction / norm(outlier_direction);
                        x_goal(i,:) = goal_region_center + (goal_dispersion + outlier_distance) * outlier_direction;
                    else
                        x_goal(i,:) = goal_region_center + goal_dispersion * (2*rand(1,2) - 1); % Regenerate normal goal
                    end
                    attempts = attempts + 1;
                end
            end
            
            if attempts >= max_attempts
                warning('Could not find valid goal for Robot %d after %d attempts', i, max_attempts);
            end
        end
    end
else
    obs_pos = [ -1.5  0.5;      % Fixed position for all the obstacles
             1.0  1.0;
             0.0  2.0;
             0.0 -3.0 ];
    obs_rad = [0.6 0.3 0.3 0.4];
    nObs = size(obs_pos,1);
end

%% --- Distributed λ2 params ---
lambda2_eps   = 0;      % First trigger from which applying the gain increase in Distributed approach
lambda2_warn  = 2.0;    % Second trigger from which applying the gain increase in Distributed approach
k_lambda_glob = 3.0;    % Gain affecting the scale factor in Distributed approach
gamma_max     = 4.0;    % Saturation value for the connectivity gain scaling in Distributed approach

% --- Extrapolate position: function parameters ---
CONFIG.extrapolation_enabled = true; % It estimates new positions from old position and new velocity (uniform motion-style)
CONFIG.extrapolation_max_age = 0.5;  % Max time (in seconds) to perform velocity extrapolation. If time is over, no extrapolation is done
CONFIG.comm_decay_rate = 0.1;        % Confidence decay per communication hop

% --- Blend estimate: function parameters ---
CONFIG.use_goal_blend = true;       % It uses Starting Position (SP) + Goal Position (GP) instead of SP + RS (Random Search)
CONFIG.sp_decay_timescale = 3.0;    % Time units for starting position decay
CONFIG.rs_decay_distance = 1.0;     % Distance units (normalized by R_glob)
CONFIG.sp_conf_max = 100;           % Starting position max confidence
CONFIG.sp_conf_min = 1;             % Starting position min confidence
CONFIG.rs_conf_max = 100;           % Random search max confidence
CONFIG.rs_conf_min = 1;             % Random search min confidence

% --- Robust consensus: function parameters ---
CONFIG.consensus_enabled = true;     % It triggers "robust_consensus" function, where lambda consensus (weighted between agents) is added to lambda estimations (by single agents)
CONFIG.outlier_threshold = 2.0;      % Reject estimates > 2σ from local mean
CONFIG.min_consensus_neighbors = 2;  % Min number of neighbours necessary to apply the consensus

% --- Build adjacency matrix: function parameters ---
CONFIG.edge_conf_threshold = 60;    % It skips potential edges between robots with a lower confidence

% --- Decentralized CBF step: function parameters ---
CONFIG.debug_estimation = true; % I triggers a detailed per-robot estimation debug
CONFIG.debug_frequency = 20;    % Interval of steps between each debugging printing window
CONFIG.max_hops = N;            % Threshold about max communication hops allowed in the algorithm
CONFIG.position_timeout = 1.0;  % Timeout set before cleaning the position data

% --- Messaging component setup ---
position_knowledge = cell(1, N);     % AD - position_knowledge -> A cell of N struct variables. The stable info acquired by each robot. Enhanced message structure: position, velocity, timestamp
for i = 1:N
    position_knowledge{i} = struct('robot_id', {}, 'position', {}, 'velocity', {}, ...
                                   'timestamp', {}, 'hop_count', {});
    position_knowledge{i}(1) = struct('robot_id', i, 'position', x(i,:), ...
                                      'velocity', v(i,:), 'timestamp', 0, 'hop_count', 0);
end

% --- λ2 estimates with uncertainty ---
lambda2_estimates = zeros(N, 1);    % Lambda estimate for a specific robot
lambda2_consensus = zeros(N, 1);    % Lambda consensus for a specific robot
lambda2_std = zeros(N, 1);          % Lambda standard deviation for a specific robot
lambda2_confidence = zeros(N, 1);   % Lambda confidence for a specific robot

% --- λ2 logs ---
lambda2_log = zeros(steps, 1);      % Mean of lambda2_consensus at each step over all the robots
lambda2_std_log = zeros(steps, 1);  % Mean of lambda2_std at each step over all the robots
lambda2_conf_log = zeros(steps, 1); % Mean of lambda2_confidence at each step over all the robots

%% --- QP setup ---
opts = optimoptions('quadprog','Display','off','Algorithm','interior-point-convex');
keys = ["Centralized", "Decentralized", "Distributed"]; % Dict used to parametrize the switch block in Simulink model
values = [1, 2, 3];
d = dictionary(keys, values);
opt_strategy = "Distributed"; % Choose one of the opt methods in "keys" array

fprintf("Initialization done!!! Chosen optimization method = %s\n", opt_strategy)