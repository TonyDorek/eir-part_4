%% INITIALIZING PARAMETERS
% Run it BEFORE executing the main script...

clear;
clc;
close all;
s = rng(0);   % Seed for the random number generation

%% --- Relative paths ---
addpath(genpath('optim'))   % Adding path to optimization scripts
addpath(genpath('utils'))   % Adding path to utility functions

%% --- Simulation parameters ---    
N    = 7;   % Number of agents
dim  = 2;   % Space dimension
dt   = 0.05;   % Timestep
Tsim = 20;   % Tot. simulation time
steps = floor(Tsim/dt);   % Tot. simulation steps

%% --- Initial positions / velocities ---
R0 = 10;   % Radius of the circle for the initial positions
theta = linspace(0,2*pi*(1-1/N),N)';   % Orientation of the initial positions
x = [R0*cos(theta), R0*sin(theta)] + 0.3*randn(N,2);   % Initial position vector (circular distribution + random component)
v = zeros(N,2);   % Initial velocity vector

%% --- Controller parameters  ---
x_goal = [0 -1];   % Nominal goal for the formation
k_p = 1.0;   % Position gain
k_d = 2.0;   % Velocity gain

%% --- Control Barrier Function parameters ---
R_glob = 5.0;   % Global communication radius (for neighborhood determination)
R_loc = 4.0;   % Maximum local communication radius (for connectivity constraint)
conn_margin = 1.0;   % Margin for connectivity constraint. Enforce it only for pairs within this margin over R_loc								 
dmin = 1.0;   % Minimum distance between robots (for agent collision constraint)
rsafe = 0.25;   % Obstacle safety margin (for obstacle collision constraint, together with the obstacle radius)									
cbf_gain_conn = 3.0;   % alpha multiplier for connectivity (alpha * h)
cbf_gain_col  = 5.0;   % idem for agent collision
cbf_gain_obs  = 4.0;   % idem for obstacle collision
Tpred = 0.5;   % Prediction horizon (seconds). Smaller = more conservative, larger = more predictive

%% --- Obstacles ---
nObs = 4;   % Number of obstacles
obs_pos = [ -1.5  0.5;   % Position for each obstacle
             1.0 1.0;
             0.0 2.0;
             0.0 -3.0];
obs_rad = [0.6 0.3 0.3 0.4];   % Obstacle radius

%% --- Logging ---
xlog = zeros(steps,N,2);   % Initializing history of the state variable
lambda2_log = zeros(steps,1);   % Initializing history of the lambda2 eigenvalue

%% --- Parameters specific for decentralized approach ---
u_prev = zeros(N,2);   % store last accelerations for neighbor prediction

%% --- Parameters specific for hybrid approach ---
special_idx = 2;   % Index to select the "special" agent
G = setdiff(1:N, special_idx);   % List of indices without the special_idx
x_goal_alt = [0 -6];   % Alternative goal for one special agent (pulls it away from group)

% --- Global λ2-based connectivity feedback (soft constraint) ---
lambda2_eps   = 0;   % Desired minimal global connectivity level
lambda2_warn  = 2.0;   % Warning threshold (under which trigger the global gain gamma)
k_lambda_glob = 3.0;   % Gain for global modulation (see the equation!)
gamma_max     = 4.0;   % Max scaling of global connectivity gain
gamma_log   = zeros(steps,1);   % Initializing history of global gain gamma
lambda_feedback_active = false; % Flag: global λ2-feedback deactivated at start

%% --- QP setup ---
opts = optimoptions('quadprog','Display','off','Algorithm','interior-point-convex');
opt_strategy = "hybrid";   % Possible values: "centralized", "decentralized", "hybrid"


