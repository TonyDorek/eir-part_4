%% INITIALIZING PARAMETERS
% Run it BEFORE executing the logic...

clear;
clc;
close all;
s = rng(0);   % Seed
%s = rng('shuffle');   % Seed for the random number generation

%% --- Starting configuration ---
CONFIG = struct();
CONFIG.randomize_robots_initpos = false; % If true, robot initial positions are randomized
CONFIG.randomize_goals = false;           % If true, robot goal positions are randomized
CONFIG.randomize_obstacles = false;      % If true, obstacle positions and sizes are randomized
CONFIG.randomize_outlier_goal = false;       % If true, one robot will have an outlier goal position

%% --- Relative paths ---
addpath(genpath('../helpers'))

%% --- Simulation parameters ---    
N    = 5;                   % Number of robots
dim  = 2;                   % Space dimension
dt   = 0.05;                % Timestep
Tsim = 10;                  % Tot. simulation time
steps = floor(Tsim/dt);     % Tot. simulation 
Tstop = Tsim;    % se il modello usa Tstop come StopTime
colors = hsv(N);
%% --- Initial positions / velocities ---
R0 = 10;                                               % Radius of the circle for the initial positions
theta = linspace(0,2*pi*(1-1/N),N)';                   % Orientation of the initial positions
v = zeros(N,2);                                        % Initial velocity vector
if CONFIG.randomize_robots_initpos
    x = [R0*cos(theta), R0*sin(theta)] + 0.5*randn(N,2);   % Initial position vector (circular distribution + random component)
else
    x = [R0*cos(theta), R0*sin(theta)];   % Initial position vector (circular distribution)
end

%% -- Goal Setting -- 
if CONFIG.randomize_goals               % IF Randomized goal positions
    goal_region_center = [0 -1];
    goal_dispersion = 4.0;              % Controls spread of goals (higher = more dispersed)
    % Generate goals
    x_goal = goal_region_center + goal_dispersion * (2*rand(N,2) - 1);  % N×2 matrix of individual goals
    % Add outlier if enabled
    outlier_idx = -1;  % Initialize
    if CONFIG.randomize_outlier_goal
        outlier_idx = randi(N);  % Random robot gets outlier goal
        outlier_direction = randn(1,2);
        outlier_direction = outlier_direction / norm(outlier_direction);
        outlier_distance = 5.0;  % Additional distance for outlier goal
        x_goal(outlier_idx,:) = goal_region_center + (goal_dispersion + outlier_distance) * outlier_direction;
        fprintf('Outlier goal assigned to Robot %d\n', outlier_idx);               % Debug info
    end
else
    x_goal = ones(N,1)*[0 -1];  % Fixed goal for all robots
    % if CONFIG.specific_outlier_goal
    %     outlier_idx = 2; 
    %     x_goal(outlier_idx,:) = [0 -6];
    %     fprintf('Outlier goal assigned to Robot %d\n', outlier_idx);
    % end
end

%% --- Controller parameters  ---
x0_vec = reshape(x', [], 1);   % pos iniziale in forma vettoriale per Simulink, 2N x 1
v0_vec = reshape(v', [], 1);   % vel iniziale, 2N x 1
k_p = 1.0;   % Position gain
k_d = 2.0;   % Velocity gain

%% --- Control Barrier Function parameters ---
R_glob = 5.0;                               % Global communication radius (for neighborhood determination)
R_loc = 4.0;                                % Maximum local communication radius (for connectivity constraint)
conn_margin = 1.0;                          % Margin for connectivity constraint. Enforce it only for pairs within this margin over R_loc                               
dmin = 1.0;                                 % Minimum distance between robots (for agent collision constraint)
rsafe = 0.25;                               % Obstacle safety margin (for obstacle collision constraint, together with the obstacle radius)                                 
cbf_gain_conn = 1.0;                        % alpha multiplier for connectivity (alpha * h)
cbf_gain_col  = 5.0;                        % idem for agent collision
cbf_gain_obs  = 4.0;                        % idem for obstacle collision
Tpred = 0.5;                                % Prediction horizon (seconds). Smaller = more conservative, larger = more predictive

%% --- Obstacles ---
if CONFIG.randomize_obstacles
    nObs = 4;   % Number of obstacles
    % Randomized obstacle positions within workspace
    obs_pos = 8 * (2*rand(nObs,2) - 1);  % Random positions in [-8,8]×[-8,8]
    obs_rad = 0.3 + 0.3*rand(nObs,1);    % Random radii between 0.3 and 0.6

    %% --- Validate and regenerate goals if too close to obstacles ---
    if CONFIG.randomize_robots_initpos
        goal_obstacle_clearance = 1.0;  % Minimum distance from obstacle edge to goal
        max_attempts = 100;

        for i = 1:N
            attempts = 0;
            valid = false;
            
            while ~valid && attempts < max_attempts
                % Check distance to all obstacles
                valid = true;
                for o = 1:nObs
                    dist_to_obs = norm(x_goal(i,:) - obs_pos(o,:)) - obs_rad(o);
                    if dist_to_obs < goal_obstacle_clearance
                        valid = false;
                        break;
                    end
                end
                
                % If invalid, regenerate this goal
                if ~valid
                    if i == outlier_idx && CONFIG.randomize_outlier_goal
                        % Regenerate outlier
                        outlier_direction = randn(1,2);
                        outlier_direction = outlier_direction / norm(outlier_direction);
                        x_goal(i,:) = goal_region_center + (goal_dispersion + outlier_distance) * outlier_direction;
                    else
                        % Regenerate normal goal
                        x_goal(i,:) = goal_region_center + goal_dispersion * (2*rand(1,2) - 1);
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
    obs_pos = [ -1.5  0.5;
             1.0  1.0;
             0.0  2.0;
             0.0 -3.0 ];
    obs_rad = [0.6 0.3 0.3 0.4];
    nObs = size(obs_pos,1);
end

%% --- Logging placeholder ---
%xlog = zeros(steps,N,2);
lambda2_log = zeros(steps,1);
gamma_log = zeros(steps,1);

%% --- Decentralized / hybrid params ---
% (qui solo parametri logici)
special_idx = 2;
G = setdiff(1:N, special_idx);
x_goal_alt = [0 -6];

lambda2_eps   = 0;
lambda2_warn  = 2.0;
k_lambda_glob = 3.0;
gamma_max     = 4.0;
lambda_feedback_active = false;   % il valore iniziale del flag
											   
%% --- Distributed lambda params ---

% --- Monte Carlo Uncertainty Quantification ---
% CONFIG.mc_samples = 50;
% CONFIG.mc_enabled = false;

% --- Velocity Extrapolation ---
CONFIG.extrapolation_enabled = true;
CONFIG.extrapolation_max_age = 0.5;  % seconds

% --- Normalized Decay Parameters (dimensionless) ---
CONFIG.sp_decay_timescale = 3.0;     % Time units for starting position decay
CONFIG.rs_decay_distance = 1.0;      % Distance units (normalized by R_glob)
CONFIG.comm_decay_rate = 0.1;        % Confidence decay per communication hop

% --- Robust Consensus ---
CONFIG.consensus_enabled = true;
CONFIG.use_goal_blend = true;        % Use SP+Goal instead of SP+RS
CONFIG.outlier_threshold = 2.0;      % Reject estimates > 2σ from local mean
CONFIG.min_consensus_neighbors = 2;

% --- Validation Heuristics ---
CONFIG.validation_enabled = true;
CONFIG.validation_frequency = 5;     % Check every N steps

% --- Confidence Thresholds ---
CONFIG.edge_conf_threshold = 60;
CONFIG.sp_conf_max = 100;
CONFIG.sp_conf_min = 1;
CONFIG.rs_conf_max = 100;
CONFIG.rs_conf_min = 1;

% --- Communication & Timeouts ---
CONFIG.max_hops = N;
CONFIG.position_timeout = 1.0;

% --- Debug Settings ---
CONFIG.debug_frequency = 20;
CONFIG.debug_estimation = true;      % Detailed per-robot estimation debug


%% --- QP setup ---
opts = optimoptions('quadprog','Display','off','Algorithm','interior-point-convex');

% Dict in order to parametrize the switch block in Simulink model
keys = ["Centralized", "Decentralized", "Hybrid", "Distributed"];
values = [1, 2, 3, 4];
d = dictionary(keys, values);

opt_strategy = "Decentralized"; % choose one of the opt methods in "keys" array