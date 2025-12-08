%% INITIALIZING PARAMETERS
% Run it BEFORE executing the Simulink model...

clear;
clc;
close all;
s = rng(0);   % Seed

%% --- Relative paths ---
addpath(genpath('optim'))
addpath(genpath('utils'))

%% --- Simulation parameters ---    
N    = 5;        % 5 ROBOT
dim  = 2;
dt   = 0.05;
Tsim = 10;
steps = floor(Tsim/dt);
Tstop = Tsim;    % se il modello usa Tstop come StopTime

%% --- Controller parameters  ---
x_goal = [0 -1];
k_p = 1.0;
k_d = 2.0;

%% --- CBF parameters ---
R_glob = 5.0;
R_loc = 4.0;
conn_margin = 1.0;
dmin = 1.0;
rsafe = 0.25;
cbf_gain_conn = 3.0;
cbf_gain_col  = 5.0;
cbf_gain_obs  = 4.0;
Tpred = 0.5;

%% --- Obstacles (scalabili) ---
obs_pos = [ -1.5  0.5;
             1.0  1.0;
             0.0  2.0;
             0.0 -3.0 ];
obs_rad = [0.6 0.3 0.3 0.4];

nObs = size(obs_pos,1);      % numero ostacoli AUTOMATICO

%% --- Logging placeholder (se ti serve lato script MATLAB) ---
xlog         = zeros(steps,N,2);
lambda2_log  = zeros(steps,1);
gamma_log    = zeros(steps,1);

%% --- Decentralized / hybrid params ---
% (qui solo parametri logici, NON stati)
special_idx = 2;
G = setdiff(1:N, special_idx);
x_goal_alt = [0 -6];

lambda2_eps   = 0;
lambda2_warn  = 2.0;
k_lambda_glob = 3.0;
gamma_max     = 4.0;
lambda_feedback_active = false;   % il valore iniziale del flag

%% --- QP setup ---
opts = optimoptions('quadprog','Display','off','Algorithm','interior-point-convex');

% opt_strategy non serve più a Simulink, ma puoi tenerlo per riferimento
opt_strategy = "centralized";   % o "decentralized", "hybrid" (solo come info)
