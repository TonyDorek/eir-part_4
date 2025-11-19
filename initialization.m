%% INITIALIZING PARAMETERS
% Run it BEFORE executing the main script...

clear;
clc;
close all;

s = rng(0);

%% --- Adding relative paths to function and scripts ---
addpath(genpath('optim'))
addpath(genpath('utils'))

%% --- Simulation parameters ------------------------------------
N    = 7;              % number of robots
dim  = 2;
dt   = 0.05;
Tsim = 20;
steps = floor(Tsim/dt);
special_idx = 2; % random index to select the "special" agent (for distributed approach)
G = setdiff(1:N, special_idx);

%% initial positions / velocities
R0 = 10;
theta = linspace(0,2*pi*(1-1/N),N)';
x = [R0*cos(theta), R0*sin(theta)] + 0.3*randn(N,2);
v = zeros(N,2);

%% --- Nominal goal controller ----------------------------------
x_goal = [0 -1];
x_goal_alt = [0 -6]; % alternative goal for one special agent (pulls it away from group)

k_p = 1.0; % position gain
k_d = 2.0; % damping

%% --- Barrier-function / prediction params ---------------------
R_comm = 5.0;       % neighborhood radius (for decentralized approach)
Rmax = 4.0;         % maximum comm range (connectivity)
conn_margin = 1.0; % margin for connectivity constraint. Enforce only for pairs within this margin of Rmax								 

dmin = 1.0;         % minimum distance between robots (collision)

rsafe = 0.25;       % obstacle safety margin												

cbf_gain_conn = 3.0;   % alpha multiplier for connectivity (alpha * h)
cbf_gain_col  = 5.0;   % collision
cbf_gain_obs  = 4.0;   % obstacle

Tpred = 0.5; % Prediction horizon (seconds). Smaller = less conservative, larger = more predictive.
u_prev = zeros(N,2); % store last accelerations for neighbor prediction (for decentralized approach)

%% --- Global λ2-based connectivity feedback (soft constraint) --------------------
lambda2_eps   = 0;      % desired minimal global connectivity level
lambda2_warn  = 2.0;      % warning threshold
k_lambda_glob = 3.0;      % gain for global modulation
gamma_max     = 4.0;      % max scaling of connectivity CBF gain

lambda_feedback_active = false; % Flag: global λ2-feedback deactivated at start

%% --- Obstacles ------------------------------------------------
nObs = 4;
obs_pos = [ -1.5  0.5;
             1.0 1.0;
             0.0 2.0;
             0.0 -3.0];
obs_rad = [0.6 0.3 0.3 0.4];

%% --- QP setup --------------------------------------------------
opts = optimoptions('quadprog','Display','off','Algorithm','interior-point-convex');

%% --- Logging ---------------------------------------------------
xlog = zeros(steps,N,2);
lambda2_log = zeros(steps,1);
gamma_log   = zeros(steps,1);

%% --- Optimization selector -------------------------------------
opt_strategy = "decentralized";  % Possible values: "centralized", "decentralized", "hybrid"
