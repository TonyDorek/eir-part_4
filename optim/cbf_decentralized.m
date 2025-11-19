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

disp("Executing script with decentralized control...")

%% --- Handling figures ---
fig = figure;

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
        u_nom_i = u_nom_fun(k_p, k_d, x(i,:), v(i,:), x_goal);

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
        thr = norm(v(i,:));
        if thr > vmax
            v(i,:) = (vmax/thr)*v(i,:);
        end
    end

    % ----- log & visualize -------------------------------------
    xlog(k,:,:) = x;

    % monitor connectivity λ₂
    Aconn = zeros(N);
    for i=1:N-1
        for j=i+1:N
            dist = norm2(x(i,:)-x(j,:));
            if dist <= R_comm
                Aconn(i,j) = incmat_com(dist, R_comm);
                Aconn(j,i) = Aconn(i,j);
            end
        end
    end
    L = diag(sum(Aconn,2)) - Aconn;
    ev = sort(eig(L));
    lambda2 = ev(min(2,length(ev)));
    lambda2_log(k)=lambda2;

    if mod(k,5)==0  % Visualize every 5 steps
        clf;
        hold on;
        if ~isvalid(fig) || ~ishandle(fig) % Check if figure is still open
            disp('Figure closed. Simulation stopped.');
            close;
            return;
        end
        
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
        scatter(x(:,1),x(:,2),80,'b','filled');  % plot agents								 
        quiver(x(:,1),x(:,2),v(:,1),v(:,2),0.4,'Color',[0 0.6 0]);  % plot agent motion directions
        axis equal; grid on; xlim([-8 8]); ylim([-8 8]);
        xlabel('x [m]'); ylabel('y [m]');
        title(sprintf('t = %.2f s  |  \\lambda_2 = %.3f',t,lambda2));
        drawnow;
    end
end

disp("Local optimization problem(s) - Resolved with success!")

