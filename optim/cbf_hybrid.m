% cbf_hybrid.m
% ---------------------------------------------------------------
% Decentralized double-integrator multi-robot system using
% predicted-distance (velocity-aware) CBFs, with:
%  - local collision / connectivity / obstacle constraints
%  - global connectivity feedback via λ2
%  - one "special" agent whose goal changes to pull it away from
%    the group once global connectivity is established (λ2 > eps).
%
% Each agent i solves a local QP:
%     min ||u_i - u_nom_i||^2
%     s.t. local CBF inequalities (hard constraints, no slacks).
%
% λ2 is computed centrally here (standing in for a distributed
% λ2-estimator that would run on the robots).
% ---------------------------------------------------------------

disp("Executing script with hybrid control (decentralized CBFs + global connectivity soft constraint...)")

%% --- Handling figures ---
fig = figure;

fprintf('Outlier agent index = %d\n', special_idx);

%% ==============================================================
for k = 1:steps
    t = (k-1)*dt;

    % --- Global connectivity evaluation (λ2) --------------------
    Aconn = zeros(N);
    for i=1:N-1
        for j=i+1:N
            dist = norm2(x(i,:)-x(j,:));
            if dist <= R_glob
                Aconn(i,j) = incmat_com(dist, R_glob);
                Aconn(j,i) = Aconn(i,j);
            end
        end
    end
    L = diag(sum(Aconn,2)) - Aconn;
    ev = sort(eig(L));
    lambda2 = ev(min(2,length(ev)));      % current global λ2
    lambda2_log(k) = lambda2;

    % As if each agent had a distributed λ2 estimate:
    lambda2_hat = lambda2;

    % --- Turn ON global λ2 feedback once λ2 >= lambda2_eps -----
    if (~lambda_feedback_active) && (lambda2_hat > lambda2_eps)
        lambda_feedback_active = true;
        fprintf('Global λ2-based feedback ACTIVATED at t = %.2f s (λ2 = %.3f).\n', t, lambda2_hat);
    end

    % --- Global connectivity gain factor γ_glob -----------------
    if lambda_feedback_active
        if lambda2_hat < lambda2_warn
            gamma_glob = 1 + k_lambda_glob * (lambda2_warn - lambda2_hat);
            gamma_glob = min(gamma_glob, gamma_max);
        else
            gamma_glob = 1.0;
        end
    else
        gamma_glob = 1.0;
    end
    %gamma_glob = 1.0;  % Uncomment to deactivate conn gain factor reinforcement
    gamma_log(k) = gamma_glob;

    % effective connectivity CBF gain
    cbf_gain_conn_eff = cbf_gain_conn * gamma_glob;

    % ---- per-agent decentralized QP ----------------------------
    U = zeros(N,2);

    for i = 1:N
        % ---- choose agent-specific goal ------------------------
        if (i == special_idx) && lambda_feedback_active
            % Once global connectivity is achieved, this agent
            % tries to go to a DIFFERENT goal (pulling away).
            goal_i = x_goal_alt;
        else
            goal_i = x_goal;
        end

        % nominal acceleration for agent i
        u_nom_i = -k_p * (x(i,:) - goal_i) - k_d * v(i,:);

        % neighbors within comm radius (decentralized sensing)
        dists = sqrt(sum((x - x(i,:)).^2,2));
        neigh = find(dists <= R_glob & (1:N)' ~= i);

        % build local constraints: A_i * u_i <= b_i
        A_i = []; b_i = [];

        for j = neigh'
            xij = x(i,:) - x(j,:);
            vij = v(i,:) - v(j,:);
            u_j = u_prev(j,:);    % use previous accel of neighbor for prediction
            dij = norm2(xij);

            % --- collision avoidance (predicted) -----------------
            xT = xij + Tpred * vij;
            h_col = (xT * xT') - dmin^2;
            % (-2*Tpred*xT)*u_i <= 2*xT*vij' - 2*Tpred*xT*u_j' + cbf_gain_col*h_col
            arow = -2*Tpred * xT;
            brow = 2*(xT * vij') - 2*Tpred*(xT * u_j') + cbf_gain_col*h_col;
            A_i = [A_i; arow];
            b_i = [b_i; brow];

            % --- local connectivity maintenance (predicted) ------
            if dij < (R_loc + conn_margin)
                xT_c = xij + Tpred * vij;
                h_conn = R_loc^2 - (xT_c * xT_c');
                % (2*Tpred*xT_c)*u_i <= -2*xT_c*vij' + 2*Tpred*xT_c*u_j' + cbf_gain_conn_eff*h_conn
                arow_c =  2*Tpred * xT_c;
                brow_c = -2*(xT_c * vij') + 2*Tpred*(xT_c * u_j') + cbf_gain_conn_eff * h_conn;
                A_i = [A_i; arow_c];
                b_i = [b_i; brow_c];
            end
        end

        % --- obstacle avoidance (predicted) ----------------------
        for o = 1:nObs
            xio = x(i,:) - obs_pos(o,:);
            vio = v(i,:);
            xT_o = xio + Tpred * vio;
            h_obs = (xT_o * xT_o') - (obs_rad(o) + rsafe)^2;
            % (-2*Tpred*xT_o)*u_i <= 2*xT_o*vio' + cbf_gain_obs*h_obs
            arow_o = -2*Tpred * xT_o;
            brow_o = 2*(xT_o * vio') + cbf_gain_obs*h_obs;
            A_i = [A_i; arow_o];
            b_i = [b_i; brow_o];
        end

        % --- solve local 2D QP: min ||u_i - u_nom_i||^2 s.t. A_i*u_i <= b_i
        H = 2*eye(2);
        f = -2*u_nom_i';
        if isempty(A_i)
            u_i = u_nom_i';
        else
            [u_i,~,flag] = quadprog(H,f,A_i,b_i,[],[],[],[],[],opts);
            if flag <= 0
                % if infeasible or failed, fallback to nominal
                u_i = u_nom_i';
            end
        end
        U(i,:) = u_i';
    end

    % integrate dynamics
    v = v + dt*U;
    x = x + dt*v;
    u_prev = U;

    % velocity saturation
    vmax = 2.0;
    for i=1:N
        s = norm(v(i,:));
        if s > vmax
            v(i,:) = (vmax/s)*v(i,:);
        end
    end

    % ----- log & visualize -------------------------------------
    xlog(k,:,:) = x;

    if mod(k,5)==0
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
        % connectivity edges (monitor)
        for i=1:N-1
            for j=i+1:N
                if Aconn(i,j)>0
                    plot([x(i,1),x(j,1)],[x(i,2),x(j,2)],'k-','LineWidth',0.5);
                end
            end
        end
        % agents
        scatter(x(:,1),x(:,2),80,'b','filled');
        % highlight special agent
        plot(x(special_idx,1), x(special_idx,2), 'ro', 'MarkerSize', 12, 'LineWidth',1.5);
        quiver(x(:,1),x(:,2),v(:,1),v(:,2),0.4,'Color',[0 0.6 0]);
        %goals
        plot(x_goal(1), x_goal(2), '-x', 'MarkerEdgeColor', 'b', 'MarkerSize', 12, 'LineWidth', 1.5)
        plot(x_goal_alt(1), x_goal_alt(2), '-x', 'MarkerEdgeColor', 'r', 'MarkerSize', 12, 'LineWidth', 1.5)
        
        axis equal; grid on; xlim([-8 8]); ylim([-8 8]);
        xlabel('x [m]'); ylabel('y [m]');
        title(sprintf('Hybrid optimization | t=%.2f s | λ2=%.3f | γ=%.2f', ...
                      t, lambda2, gamma_glob));
        drawnow;
    end
end

dist_main = vecnorm(x(G,:) - x_goal, 2, 2);  % distance for all main agents
d_main = mean(dist_main);                  % average

d_out = norm(x(special_idx,:) - x_goal_alt);  % --- Outlier accuracy ---

disp("Hybrid optimization problem(s) - Resolved with success!")
