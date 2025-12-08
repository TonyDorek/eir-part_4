%% INITIALIZING PARAMETERS
% Run it BEFORE executing the Simulink model...

clear;
clc;
close all;
s = rng(0);   % Seed

%% --- Relative paths ---
addpath(genpath('../functions'))

%% --- Simulation parameters ---    
N    = 5;        % <<< ORA 5 ROBOT
dim  = 2;
dt   = 0.05;
Tsim = 10;
steps = floor(Tsim/dt);

%% --- Initial positions / velocities ---
R0 = 10;
theta = linspace(0,2*pi*(1-1/N),N)';   
x = [R0*cos(theta), R0*sin(theta)] + 0.3*randn(N,2);
v = zeros(N,2);

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

%% --- Obstacles (SCALABILI) ---
% Qui puoi mettere quanti ostacoli vuoi: nObs = size(obs_pos,1)
obs_pos = [ -1.5  0.5;
             1.0  1.0;
             0.0  2.0;
             0.0 -3.0 ];     % puoi aggiungere/rimuovere righe
obs_rad = [0.6 0.3 0.3 0.4]; % stessa lunghezza di obs_pos

nObs = size(obs_pos,1);      % <<< numero ostacoli AUTOMATICO

%% --- Logging placeholder (se ti serve ancora lato script) ---
xlog = zeros(steps,N,2);
lambda2_log = zeros(steps,1);

%% --- Decentralized / hybrid params (se li usi più avanti) ---
u_prev = zeros(N,2);
special_idx = 2;
G = setdiff(1:N, special_idx);
x_goal_alt = [0 -6];

lambda2_eps   = 0;
lambda2_warn  = 2.0;
k_lambda_glob = 3.0;
gamma_max     = 4.0;
gamma_log   = zeros(steps,1);
lambda_feedback_active = false;

%% --- QP setup ---
opts = optimoptions('quadprog','Display','off','Algorithm','interior-point-convex');

% Dict in order to parametrize the switch block in Simulink model
keys = ["centralized", "decentralized", "hybrid"];
values = [1, 2, 3];
d = dictionary(keys, values);

opt_strategy = "hybrid";

%% --- Stato iniziale in forma vettoriale per Simulink ---
x0_vec = reshape(x', [], 1);   % 2N x 1
v0_vec = reshape(v', [], 1);   % 2N x 1

Tstop = Tsim;
