% connectivity_CBF_doubleintegrator_predicted.m
% ---------------------------------------------------------------
% Double-integrator multi-robot system using velocity-aware CBFs.
% Barriers use predicted relative positions: x + T*v
% Each step solves a QP:
%     min ||u - u_nom||^2
%     s.t. linear CBF inequalities (affine in u)
%
% Dynamics:  xdot = v ,  vdot = u  (acceleration control)
%
% ---------------------------------------------------------------

clear; close all; rng(2);

%% --- Simulation parameters ------------------------------------
N    = 10;              % number of robots
dim  = 2;
dt   = 0.05;
Tsim = 10;
steps = floor(Tsim/dt);

% initial positions / velocities
R0 = 3;
theta = linspace(0,2*pi*(1-1/N),N)';
x = [R0*cos(theta), R0*sin(theta)] + 0.3*randn(N,2);
v = zeros(N,2);

%% --- Nominal goal controller ----------------------------------
x_goal = [4 0];
k_p = 1.0;         % position gain
k_d = 2*sqrt(k_p); % damping
% nominal accel (PD to goal)
u_nom_fun = @(xi,vi) -k_p*(xi - x_goal) - k_d*vi;

%% --- Barrier-function / prediction params ---------------------
Rmax = 6.0;         % maximum comm range (connectivity)
dmin = 0.6;         % minimum distance between robots (collision)
rsafe = 0.25;       % obstacle safety margin

cbf_gain_conn = 1.0;   % alpha multiplier for connectivity (alpha * h)
cbf_gain_col  = 6.0;   % collision
cbf_gain_obs  = 6.0;   % obstacle

% Prediction horizon (seconds). Smaller = less conservative, larger = more predictive.
Tpred = 0.5;

% Only enforce connectivity for pairs within this margin of Rmax
conn_margin = 0.5;

%% --- Obstacles ------------------------------------------------
nObs = 2;
obs_pos = [ -1.5  0.5;
             1.0 0.0 ];
obs_rad = [0.6 0.3];

%% --- QP setup --------------------------------------------------
opts = optimoptions('quadprog','Display','off',...
                    'Algorithm','interior-point-convex','MaxIterations',200);

vecIdx = @(i) (2*(i-1)+1 : 2*i);   % 2D indices for agent i
norm2  = @(z) sqrt(sum(z.^2));

%% --- Logging ---------------------------------------------------
xlog = zeros(steps,N,2);
lambda2_log = zeros(steps,1);

%% ==============================================================
for k = 1:steps
    t = (k-1)*dt;

    % Nominal accelerations
    u_nom = zeros(2*N,1);
    for i=1:N
        u_nom(vecIdx(i)) = u_nom_fun(x(i,:),v(i,:))';
    end

    % Build linear constraints in the form Arows * u <= brows
    Arows = [];
    brows = [];

    % Precompute pairwise distances and relative velocities
    for i=1:N-1
        for j=i+1:N
            xij = x(i,:) - x(j,:);        % relative position
            vij = v(i,:) - v(j,:);        % relative velocity
            dij = norm2(xij);

            %% --- Collision avoidance (predicted) -----------------
            % Use predicted relative position: xT = xij + Tpred * vij
            xT = xij + Tpred * vij;       % 1x2
            h_col = (xT * xT') - dmin^2;  % predicted squared dist minus threshold
            % derivative: dh/dt = 2 xT' * (vij + Tpred*(u_i - u_j))
            % enforce: dh/dt + cbf_gain_col * h_col >= 0
            % -> 2*Tpred * xT'*(u_i - u_j) >= - 2*xT'*vij - cbf_gain_col * h_col
            % rewrite as linear inequality A*u <= b:
            % (-2*Tpred*xT)' * u_i + (2*Tpred*xT)' * u_j <= 2*xT'*vij + cbf_gain_col*h_col
            if true
                arow = zeros(1, 2*N);
                vec = -2 * Tpred * xT;   % 1x2
                arow(vecIdx(i)) = vec;
                arow(vecIdx(j)) = -vec;
                brow = 2*(xT * vij') + cbf_gain_col * h_col;
                % numerical safety: if xT ~ 0, constraint becomes trivial; allow small epsilon
                Arows = [Arows; arow];
                brows = [brows; brow];
            end

            %% --- Connectivity maintenance (predicted) --------------
            % We would like predicted distance <= Rmax, i.e. h_conn = Rmax^2 - ||xT||^2 >= 0
            % depending on your behavior, you may enforce this only when near the boundary
            if dij < (Rmax + conn_margin)
                xT_conn = xij + Tpred * vij;
                h_conn = Rmax^2 - (xT_conn * xT_conn');
                % dh/dt = -2 xT_conn' * (vij + Tpred*(u_i - u_j))
                % enforce: dh/dt + cbf_gain_conn * h_conn >= 0
                % -> -2*Tpred*xT_conn'*(u_i - u_j) >= 2*xT_conn'*vij - cbf_gain_conn*h_conn
                % rewrite: (2*Tpred*xT_conn')*u_i + (-2*Tpred*xT_conn')*u_j <= -2*xT_conn'*vij + cbf_gain_conn*h_conn
                arow = zeros(1, 2*N);
                vecc = 2 * Tpred * xT_conn;  % 1x2
                arow(vecIdx(i)) =  vecc;
                arow(vecIdx(j)) = -vecc;
                brow = -2*(xT_conn * vij') + cbf_gain_conn * h_conn;
                Arows = [Arows; arow];
                brows = [brows; brow];
            end

        end
    end

    % --- Obstacle avoidance (predicted) -------------------------
    for i=1:N
        for o=1:nObs
            xio = x(i,:) - obs_pos(o,:);
            vio = v(i,:);
            xT_o = xio + Tpred * vio;   % predicted relative pos to obstacle
            h_obs = (xT_o * xT_o') - (obs_rad(o) + rsafe)^2;
            % dh/dt = 2 xT_o' * (vio + Tpred * u_i)
            % enforce dh/dt + cbf_gain_obs * h_obs >= 0
            % -> 2*Tpred * xT_o' * u_i >= -2 * xT_o' * vio - cbf_gain_obs * h_obs
            % rewrite: (-2*Tpred*xT_o)' * u_i <= 2*(xT_o * vio') + cbf_gain_obs * h_obs
            arow = zeros(1,2*N);
            vec = -2 * Tpred * xT_o;
            arow(vecIdx(i)) = vec;
            brow = 2 * (xT_o * vio') + cbf_gain_obs * h_obs;
            Arows = [Arows; arow];
            brows = [brows; brow];
        end
    end

    % --- Solve QP:  min ||u - u_nom||^2 subject to A*u <= b ----
    H = 2*eye(2*N);
    f = -2*u_nom;
    if isempty(Arows)
        u_sol = u_nom;
    else
        % numerical regularization for H to keep solver stable
        H = H + 1e-6 * eye(2*N);
        [u_sol,~,exitflag] = quadprog(H,f,Arows,brows,[],[],[],[],[],opts);
        if exitflag <= 0
            warning('QP infeasible at step %d (exitflag=%d), using nominal.', k, exitflag);
            u_sol = u_nom;
        end
    end
    U = reshape(u_sol,2,[])';   % N×2 accelerations

    % integrate double-integrator dynamics
    v = v + dt*U;
    x = x + dt*v;

    % optional velocity saturation
    vmax = 2.0;
    for i=1:N
        s = norm(v(i,:));
        if s > vmax
            v(i,:) = (vmax/s)*v(i,:);
        end
    end

    % ----- log & visualize -------------------------------------
    xlog(k,:,:) = x;

    % monitor simple distance-based connectivity (for λ2 plot)
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
        title(sprintf('t = %.2f s  |  \\lambda_2 = %.3f',t,lambda2));
        drawnow;
    end
end

%% --- Plot λ₂ ---------------------------------------------------
figure;
plot((0:steps-1)*dt,lambda2_log,'LineWidth',1.4);
xlabel('time [s]'); ylabel('\lambda_2 (connectivity)'); grid on;
title('Connectivity evolution');
