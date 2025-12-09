%% INITIALIZING PARAMETERS
% Run it BEFORE executing the main script...

clear;
clc;
close all;
s = rng('shuffle');   % Seed for the random number generation

%% --- QP setup ---
opts = optimoptions('quadprog','Display','off','Algorithm','interior-point-convex');
opt_strategy = "decentralized_lambda";   % Possible values: "centralized", "decentralized", "hybrid", "decentralized_lambda"

%% --- Configuration ---
CONFIG = struct();
CONFIG.randomize_robots_initpos = true; % If true, robot initial positions are randomized
CONFIG.randomize_goals = true;           % If true, robot goal positions are randomized
CONFIG.randomize_obstacles = true;      % If true, obstacle positions and sizes are randomized
CONFIG.random_outlier_goal = true;       % If true, one robot will have an outlier goal position


%% --- Relative paths ---
addpath(genpath('optim'))   % Adding path to optimization scripts
addpath(genpath('utils'))   % Adding path to utility functions

%% --- Simulation parameters ---    
N    = 5;                   % Number of robots
dim  = 2;                   % Space dimension
dt   = 0.05;                % Timestep
Tsim = 20;                  % Tot. simulation time
steps = floor(Tsim/dt);     % Tot. simulation 

Tstop = Tsim;               %** se il modello usa Tstop come StopTime


%% --- Initial positions / velocities ---

if CONFIG.randomize_robots_initpos
    % Randomized initial positions within a circle
    R0 = 10;                                               % Radius of the circle for the initial positions
    theta = linspace(0,2*pi*(1-1/N),N)';                   % Orientation of the initial positions
    x = [R0*cos(theta), R0*sin(theta)] + 0.3*randn(N,2);   % Initial position vector (circular distribution + random component)
    v = zeros(N,2);                                        % Initial velocity vector
else
    % Fixed initial positions??
end

%% -- Goal Setting -- 
if CONFIG.randomize_goals               % IF Randomized goal positions
    goal_region_center = [0 -1];
    goal_dispersion = 4.0;              % Controls spread of goals (higher = more dispersed)
    outlier_distance = 5.0;             % Additional distance for outlier goal (if present)
    % Generate goals
    x_goal = goal_region_center + goal_dispersion * (2*rand(N,2) - 1);  % N×2 matrix of individual goals
    % Add outlier if enabled
    outlier_idx = -1;  % Initialize
    if CONFIG.random_outlier_goal
        outlier_idx = randi(N);  % Random robot gets outlier goal
        outlier_direction = randn(1,2);
        outlier_direction = outlier_direction / norm(outlier_direction);
        x_goal(outlier_idx,:) = goal_region_center + (goal_dispersion + outlier_distance) * outlier_direction;
        fprintf('Outlier goal assigned to Robot %d\n', outlier_idx);               % Debug info
    end
else
    x_goal = [0 -1];  % Fixed goal for all robots
    x_goal_alt = [0 -6];
    special_idx = 2;
    G = setdiff(1:N, special_idx);
end


%% --- Controller parameters  ---
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
                    if i == outlier_idx && CONFIG.random_outlier_goal
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
    nObs = size(obs_pos,1);      % numero ostacoli AUTOMATICO
end







%% --- Logging ---
xlog = zeros(steps,N,2);            % Initializing history of the state variable
lambda2_log = zeros(steps,1);       % Initializing history of the lambda2 eigenvalue
gamma_log   = zeros(steps,1);       % Initializing history of global gain gamma

%% --- Parameters specific for decentralized approach ---
u_prev = zeros(N,2);   % store last accelerations for neighbor prediction
%% --- Parameters specific for hybrid approach ---
% --- Global λ2-based connectivity feedback (soft constraint) ---
lambda2_eps   = 0;     % Desired minimal global connectivity level
lambda2_warn  = 2.0;   % Warning threshold (under which trigger the global gain gamma)
k_lambda_glob = 3.0;   % Gain for global modulation (see the equation!)
gamma_max     = 4.0;   % Max scaling of global connectivity gain

lambda_feedback_active = false; % Flag: global λ2-feedback deactivated at start