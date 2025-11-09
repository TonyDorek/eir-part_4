% connectivity_CBF_doubleintegrator_decentralized.m
% ---------------------------------------------------------------
% Decentralized double-integrator multi-robot system
% using predicted-distance (velocity-aware) Control Barrier Functions.
%
% Each agent solves its own 2D QP:
%     min ||u_i - u_nom_i||^2
%     s.t. local CBF inequalities for collision, connectivity, obstacles.
%
% Dynamics:  xdot = v ,  vdot = u  (acceleration control)
%
% ---------------------------------------------------------------

clear; close all; rng(2);

%% --- Simulation parameters ------------------------------------
N    = 8;              % number of robots
dim  = 2;
dt   = 0.05;
Tsim = 10;
steps = floor(Tsim/dt);

% initial positions / velocities
R0 = 12;
theta = linspace(0,2*pi*(1-1/N),N)';
x = [R0*cos(theta), R0*sin(theta)] + 0.3*randn(N,2);
v = zeros(N,2);

%% --- Nominal goal controller ----------------------------------
x_goal = [2 0];
k_p = 1.0;
k_d = 2*sqrt(k_p);
u_nom_fun = @(xi,vi) -k_p*(xi - x_goal) - k_d*vi;

%% --- Barrier-function / prediction params ---------------------
Rmax = 6.0;         % maximum comm range (connectivity)
R_comm = 6.5;       % neighborhood radius
dmin = 0.6;         % minimum distance between robots (collision)
rsafe = 0.25;       % obstacle safety margin
conn_margin = 0.8;  % margin for connectivity constraint

Tpred = 0.4;        % prediction horizon
cbf_gain_col  = 5.0;
cbf_gain_conn = 3.0;
cbf_gain_obs  = 4.0;

%% --- Obstacles ------------------------------------------------
nObs = 4;
obs_pos = [ -1.5  0.5;
             1.0 1.0;
             0.0 2.0;
             0.0 -3.0];
obs_rad = [0.6 0.3 0.3 0.4];

%% --- QP setup --------------------------------------------------
opts = optimoptions('quadprog','Display','off','Algorithm','interior-point-convex');

norm2  = @(z) sqrt(sum(z.^2));
vecIdx = @(i) (2*(i-1)+1 : 2*i);

% store last accelerations for neighbor prediction
u_prev = zeros(N,2);

%% --- Logging ---------------------------------------------------
xlog = zeros(steps,N,2);
lambda2_log = zeros(steps,1);

%% ==============================================================
for k = 1:steps
    t = (k-1)*dt;
    U = zeros(N,2);

    % ---- per-agent decentralized QP ----
    for i = 1:N
        % neighbors within comm radius
        dists = sqrt(sum((x - x(i,:)).^2,2));
        neigh = find(dists <= R_comm & (1:N)' ~= i);

        % nominal acceleration
        u_nom_i = u_nom_fun(x(i,:), v(i,:));

        % build local constraints: A_i * u_i <= b_i
        A_i = []; b_i = [];

        for j = neigh'
            xij = x(i,:) - x(j,:);
            vij = v(i,:) - v(j,:);
            u_j = u_prev(j,:);    % use previous accel of neighbor
            dij = norm2(xij);

            % --- collision avoidance (predicted) ---
            xT = xij + Tpred * vij;
            h_col = (xT * xT') - dmin^2;
            arow = -2*Tpred * xT; % row for u_i
            brow = 2*(xT * vij') - 2*Tpred*(xT * u_j') + cbf_gain_col*h_col;
            A_i = [A_i; arow];
            b_i = [b_i; brow];

            % --- connectivity maintenance (predicted) ---
            if dij < (Rmax + conn_margin)
                xT_c = xij + Tpred * vij;
                h_conn = Rmax^2 - (xT_c * xT_c');
                arow_c =  2*Tpred * xT_c;
                brow_c = -2*(xT_c * vij') + 2*Tpred*(xT_c * u_j') + cbf_gain_conn * h_conn;
                A_i = [A_i; arow_c];
                b_i = [b_i; brow_c];
            end
        end

        % --- obstacle avoidance (predicted) ---
        for o = 1:nObs
            xio = x(i,:) - obs_pos(o,:);
            vio = v(i,:);
            xT_o = xio + Tpred * vio;
            h_obs = (xT_o * xT_o') - (obs_rad(o) + rsafe)^2;
            arow_o = -2*Tpred * xT_o;
            brow_o = 2*(xT_o * vio') + cbf_gain_obs*h_obs;
            A_i = [A_i; arow_o];
            b_i = [b_i; brow_o];
        end

        % --- solve local 2D QP ---
        H = 2*eye(2);
        f = -2*u_nom_i';
        if isempty(A_i)
            u_i = u_nom_i';
        else
            [u_i,~,flag] = quadprog(H,f,A_i,b_i,[],[],[],[],[],opts);
            if flag <= 0
                u_i = u_nom_i';
            end
        end
        U(i,:) = u_i';
    end

    % integrate
    v = v + dt*U;
    x = x + dt*v;
    u_prev = U;

    % optional velocity saturation
    vmax = 2.0;
    for i=1:N
        s = norm(v(i,:));
        if s > vmax, v(i,:) = (vmax/s)*v(i,:); end
    end

    % ----- log & visualize -------------------------------------
    xlog(k,:,:) = x;

    % monitor connectivity λ₂
    Aconn = zeros(N);
    for i=1:N-1
        for j=i+1:N
            if norm2(x(i,:)-x(j,:)) <= Rmax
                Aconn(i,j)=1;Aconn(j,i)=1;
            end
        end
    end
    L = diag(sum(Aconn,2)) - Aconn;
    ev = sort(eig(L));
    lambda2 = ev(min(2,length(ev)));
    lambda2_log(k)=lambda2;

    if mod(k,3)==0
        clf; hold on;
        % obstacles
        for o=1:nObs
            viscircles(obs_pos(o,:),obs_rad(o),'Color','r');
            viscircles(obs_pos(o,:),obs_rad(o)+rsafe,'Color',[1 0.7 0.7],'LineStyle',':');
        end
        % edges
        for i=1:N-1
            for j=i+1:N
                if Aconn(i,j)>0
                    plot([x(i,1),x(j,1)],[x(i,2),x(j,2)],'k-','LineWidth',0.5);
                end
            end
        end
        scatter(x(:,1),x(:,2),80,'b','filled');
        quiver(x(:,1),x(:,2),v(:,1),v(:,2),0.4,'Color',[0 0.6 0]);
        axis equal; grid on; xlim([-8 8]); ylim([-8 8]);
        title(sprintf('Decentralized | t = %.2f s  |  \\lambda_2 = %.3f',t,lambda2));
        drawnow;
    end
end

%% --- Plot λ₂ ---------------------------------------------------
figure;
plot((0:steps-1)*dt,lambda2_log,'LineWidth',1.4);
xlabel('time [s]'); ylabel('\lambda_2 (connectivity)'); grid on;
title('Connectivity evolution (decentralized)');

