% ========================================================================
%  DECENTRALIZED CBF CONTROL WITH COMPREHENSIVE λ2 ESTIMATION
% ========================================================================
%
% TABLE OF CONTENTS:
%   3. INITIALIZATION
%   4. MAIN SIMULATION LOOP
%      4.1 Position Sharing & Communication
%      4.2 λ2 Estimation & Uncertainty Quantification
%      4.3 Validation & Consensus
%      4.4 Control (CBF-QP)
%      4.5 Dynamics Integration
%      4.6 Visualization
%   5. FINAL REPORT
%
% Features:
%   - Velocity-based position extrapolation
%   - Validation heuristics
%   - Monte Carlo uncertainty quantification
%   - Normalized decay parameters
%   - Robust consensus with outlier rejection
%   - Phantom edge filtering
%
% ========================================================================

disp("╔════════════════════════════════════════════════════════════════╗");
disp("║  Executing COMPREHENSIVE λ2 Estimation with CBF Control        ║");
disp("╚════════════════════════════════════════════════════════════════╝");
disp(" ");


%% ========================================================================
%%  3. INITIALIZATION
%% ========================================================================

%fig = figure;
x_start = x;
t_start = 0;

% Enhanced message structure: position, velocity, timestamp
for i = 1:N
    position_knowledge{i} = struct('robot_id', {}, 'position', {}, 'velocity', {}, ...
                                   'timestamp', {}, 'hop_count', {});
    position_knowledge{i}(1) = struct('robot_id', i, 'position', x(i,:), ...
                                      'velocity', v(i,:), 'timestamp', 0, 'hop_count', 0);
end

% colors = hsv(N);

% λ2 estimates with uncertainty
lambda2_estimates = zeros(N, 1);
lambda2_confidence = zeros(N, 1);
lambda2_std = zeros(N, 1);
lambda2_consensus = zeros(N, 1);

% Logs
lambda2_log = zeros(steps, 1);
lambda2_conf_log = zeros(steps, 1);
lambda2_std_log = zeros(steps, 1);
validation_flags = zeros(steps, N);

%% ========================================================================
%%  4. MAIN SIMULATION LOOP
%% ========================================================================

for k = 1:steps
    t = (k-1)*dt;
    
    
    %% ====================================================================
    %%  4.1 POSITION SHARING & MULTI-HOP COMMUNICATION
    %% ====================================================================
    
    % Update own position & velocity in knowledge base
    new_messages = cell(N, 1);
    for i = 1:N
        own_idx = find([position_knowledge{i}.robot_id] == i, 1);
        position_knowledge{i}(own_idx).position = x(i,:);
        position_knowledge{i}(own_idx).velocity = v(i,:);
        position_knowledge{i}(own_idx).timestamp = t;
        new_messages{i} = position_knowledge{i};
    end
    
    % Multi-hop message forwarding
    for i = 1:N
        dists = sqrt(sum((x - x(i,:)).^2, 2));
        neigh = find(dists <= R_glob & (1:N)' ~= i);
        
        for j = neigh'
            incoming_messages = new_messages{j};
            
            for msg_idx = 1:length(incoming_messages)
                msg_data = incoming_messages(msg_idx);
                robot_id = msg_data.robot_id;
                
                existing_idx = find([position_knowledge{i}.robot_id] == robot_id, 1);
                should_update = false;
                new_hop_count = msg_data.hop_count + 1;
                
                if isempty(existing_idx)
                    if new_hop_count <= CONFIG.max_hops
                        should_update = true;
                    end
                else
                    existing = position_knowledge{i}(existing_idx);
                    if (msg_data.timestamp > existing.timestamp) && (new_hop_count <= CONFIG.max_hops)
                        should_update = true;
                    end
                end
                
                if should_update
                    new_entry = struct('robot_id', robot_id, ...
                                      'position', msg_data.position, ...
                                      'velocity', msg_data.velocity, ...
                                      'timestamp', msg_data.timestamp, ...
                                      'hop_count', new_hop_count);
                    
                    if isempty(existing_idx)
                        position_knowledge{i}(end+1) = new_entry;
                    else
                        position_knowledge{i}(existing_idx) = new_entry;
                    end
                end
            end
        end
        
        % Clean stale data
        fresh_mask = ([position_knowledge{i}.timestamp] >= (t - CONFIG.position_timeout));
        position_knowledge{i} = position_knowledge{i}(fresh_mask);
    end
    
    
    %% ====================================================================
    %%  4.2 λ2 ESTIMATION & UNCERTAINTY QUANTIFICATION
    %% ====================================================================
    
    for i = 1:N
        % Identify direct neighbors and known robots
        dists_to_i = sqrt(sum((x - x(i,:)).^2, 2));
        direct_neigh = find(dists_to_i <= R_glob & (1:N)' ~= i);
        known_robots = [position_knowledge{i}.robot_id];
        
        % --- Estimate all robot positions ---
        [positions_estimated, velocities_estimated, position_confidences, position_std] = ...
            estimate_robot_positions(N, i, x, v, direct_neigh, position_knowledge, ...
                                    t, x_start, t_start, x_goal, R_glob, CONFIG);
        
        % --- Build adjacency matrix with phantom edge filtering ---
        [Aconn_local, n_edges_total, n_edges_filtered] = build_adjacency_matrix(...
            N, positions_estimated, position_confidences, known_robots, ...
            direct_neigh, i, R_glob, CONFIG.edge_conf_threshold);
        
        % --- Find connected component ---
        component_robots = find_connected_component(Aconn_local, N, i);
        n_component = length(component_robots);
        
        % Deterministic computation
        if n_component >= 2
            A_comp = Aconn_local(component_robots, component_robots);
            L_comp = diag(sum(A_comp, 2)) - A_comp;
            ev_comp = sort(eig(L_comp));
            lambda2_estimates(i) = ev_comp(2);
            lambda2_std(i) = 0.01 * lambda2_estimates(i);
        else
            lambda2_estimates(i) = 0;
            lambda2_std(i) = 0;
        end
        
        avg_conf = mean(position_confidences(component_robots));
        lambda2_confidence(i) = avg_conf * (n_component / N);
        
        % --- Debug output ---
        if CONFIG.debug_estimation && mod(k, CONFIG.debug_frequency) == 0
            % Calculate debug statistics
            n_direct = length(direct_neigh);
            n_comm = 0;
            n_blend = 0;
            for j = 1:N
                if j ~= i && ~ismember(j, direct_neigh)
                    comm_idx = find(known_robots == j, 1);
                    if ~isempty(comm_idx) && position_confidences(j) >= CONFIG.edge_conf_threshold
                        n_comm = n_comm + 1;
                    else
                        n_blend = n_blend + 1;
                    end
                end
            end
            local_deg = sum(Aconn_local(i,:)>0);
            
            print_estimation_debug(i, t, n_direct, n_comm, n_blend, ...
                n_edges_filtered, n_edges_total, n_component, N, local_deg, ...
                Aconn_local, component_robots, lambda2_estimates(i), lambda2_std(i), ...
                lambda2_confidence(i), lambda2_consensus(i), CONFIG, validation_flags(k,i));
        end
    end
    
    
    %% ====================================================================
    %%  4.3 VALIDATION & CONSENSUS
    %% ====================================================================
    
    % --- Validation heuristics ---
    if CONFIG.validation_enabled && mod(k, CONFIG.validation_frequency) == 0
        for i = 1:N
            dists_to_i = sqrt(sum((x - x(i,:)).^2, 2));
            direct_neigh = find(dists_to_i <= R_glob & (1:N)' ~= i);
            known_robots = [position_knowledge{i}.robot_id];
            
            [positions_estimated, ~, position_confidences, ~] = ...
                estimate_robot_positions(N, i, x, v, direct_neigh, position_knowledge, ...
                                        t, x_start, t_start, x_goal, R_glob, CONFIG);
            
            [Aconn_local, ~, ~] = build_adjacency_matrix(...
                N, positions_estimated, position_confidences, known_robots, ...
                direct_neigh, i, R_glob, CONFIG.edge_conf_threshold);
            
            component_robots = find_connected_component(Aconn_local, N, i);
            n_component = length(component_robots);
            local_degree = sum(Aconn_local(i, :) > 0);
            
            validation_flags(k, i) = validate_lambda2(...
                lambda2_estimates(i), i, n_component, local_degree, ...
                Aconn_local, component_robots, CONFIG);
        end
    end
    
    % --- Robust consensus ---
    lambda2_consensus = lambda2_estimates;  % Initialize
    if CONFIG.consensus_enabled
        for i = 1:N
            lambda2_consensus(i) = robust_consensus(...
                i, lambda2_estimates(i), lambda2_estimates, lambda2_confidence, ...
                x, R_glob, CONFIG);
        end
    end
    
    % --- Network statistics ---
    lambda2_avg = mean(lambda2_consensus);
    lambda2_std_avg = mean(lambda2_std);
    lambda2_conf_avg = mean(lambda2_confidence);
    
    lambda2_log(k) = lambda2_avg;
    lambda2_std_log(k) = lambda2_std_avg;
    lambda2_conf_log(k) = lambda2_conf_avg;
    
    
    %% ====================================================================
    %%  4.4 CONTROL (UNCERTAINTY-AWARE CBF-QP)
    %% ====================================================================
    
    U = zeros(N,2);
    
    for i = 1:N
        lambda2_hat_i = lambda2_consensus(i);
        lambda2_unc_i = lambda2_std(i);
        lambda2_conf_i = lambda2_confidence(i);
        
        % Conservative λ2 estimate (subtract uncertainty)
        lambda2_conservative = max(0, lambda2_hat_i - 2*lambda2_unc_i);
        
        % Modulate CBF gain based on conservative λ2
        if lambda2_conservative > lambda2_eps && lambda2_conf_i > CONFIG.edge_conf_threshold
            if lambda2_conservative < lambda2_warn
                scale_factor = 1 + k_lambda_glob * (lambda2_warn - lambda2_conservative);
                cbf_gain_conn_eff = cbf_gain_conn * scale_factor;
                cbf_gain_conn_eff = min(cbf_gain_conn_eff, cbf_gain_conn * gamma_max);
            else
                cbf_gain_conn_eff = cbf_gain_conn;
            end
            
            % Widen safety margin when uncertainty is high
            if lambda2_unc_i > 0.1
                cbf_gain_conn_eff = cbf_gain_conn_eff * (1 + lambda2_unc_i);
            end
        else
            cbf_gain_conn_eff = cbf_gain_conn * 0.5;
        end
        
        % Nominal control
        goal_i = x_goal(i,:);
        u_nom_i = -k_p * (x(i,:) - goal_i) - k_d * v(i,:);
        u_nom_i = reshape(u_nom_i, [1, 2]);  % Ensure row vector
        
        % Build CBF constraints
        dists = sqrt(sum((x - x(i,:)).^2,2));
        neigh = find(dists <= R_glob & (1:N)' ~= i);
        
        A_i = [];
        b_i = [];
        
        % Collision avoidance & connectivity maintenance
        for j = neigh'
            xij = x(i,:) - x(j,:);
            vij = v(i,:) - v(j,:);
            u_j = u_prev(j,:);
            dij = norm(xij);
            
            % Collision avoidance
            xT = xij + Tpred * vij;
            h_col = (xT * xT') - dmin^2;
            arow = -2*Tpred * xT;
            brow = 2*(xT * vij') - 2*Tpred*(xT * u_j') + cbf_gain_col*h_col;
            A_i = [A_i; arow];
            b_i = [b_i; brow];
            
            % Connectivity maintenance
            if dij < (R_loc + conn_margin)
                xT_c = xij + Tpred * vij;
                h_conn = R_loc^2 - (xT_c * xT_c');
                arow_c =  2*Tpred * xT_c;
                brow_c = -2*(xT_c * vij') + 2*Tpred*(xT_c * u_j') + cbf_gain_conn_eff * h_conn;
                A_i = [A_i; arow_c];
                b_i = [b_i; brow_c];
            end
        end
        
        % Obstacle avoidance
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
        
        % Solve QP
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
    
    
    %% ====================================================================
    %%  4.5 DYNAMICS INTEGRATION
    %% ====================================================================
    
    v = v + dt*U;
    x = x + dt*v;
    u_prev = U;
    
    % Velocity saturation
    vmax = 2.0;
    for i=1:N
        s = norm(v(i,:));
        if s > vmax
            v(i,:) = (vmax/s)*v(i,:);
        end
    end
    
    xlog(k,:,:) = x;
    vlog(k,:,:) = v;
end

disp("✓ Execution completed!")