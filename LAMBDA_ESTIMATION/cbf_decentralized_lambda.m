% ========================================================================
%  DECENTRALIZED CBF CONTROL WITH COMPREHENSIVE λ2 ESTIMATION
% ========================================================================
%
% TABLE OF CONTENTS:
%   1. CONFIGURATION & PARAMETERS
%   2. HELPER FUNCTIONS
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
%%  1. CONFIGURATION & PARAMETERS
%% ========================================================================

% --- Monte Carlo Uncertainty Quantification ---
CONFIG.mc_samples = 50;
CONFIG.mc_enabled = false;

% --- Velocity Extrapolation ---
CONFIG.extrapolation_enabled = true;
CONFIG.extrapolation_max_age = 0.5;  % seconds

% --- Normalized Decay Parameters (dimensionless) ---
CONFIG.sp_decay_timescale = 3.0;     % Time units for starting position decay
CONFIG.rs_decay_distance = 1.0;      % Distance units (normalized by R_glob)
CONFIG.comm_decay_rate = 0.1;        % Confidence decay per communication hop

% --- Robust Consensus ---
CONFIG.consensus_enabled = true;
CONFIG.use_goal_blend = true;        % Use SP+Goal instead of SP+RS
CONFIG.outlier_threshold = 2.0;      % Reject estimates > 2σ from local mean
CONFIG.min_consensus_neighbors = 2;

% --- Validation Heuristics ---
CONFIG.validation_enabled = true;
CONFIG.validation_frequency = 5;     % Check every N steps

% --- Confidence Thresholds ---
CONFIG.edge_conf_threshold = 60;
CONFIG.sp_conf_max = 100;
CONFIG.sp_conf_min = 1;
CONFIG.rs_conf_max = 100;
CONFIG.rs_conf_min = 1;

% --- Communication & Timeouts ---
CONFIG.max_hops = N;
CONFIG.position_timeout = 1.0;

% --- Debug Settings ---
CONFIG.debug_frequency = 20;
CONFIG.debug_estimation = true;      % Detailed per-robot estimation debug


%% ========================================================================
%%  2. HELPER FUNCTIONS
%% ========================================================================

% --- Smooth Connectivity Function ---
function w = incmat_com(d, R)
    % Smooth connectivity weight based on distance
    % Returns exponential decay for d <= R, zero otherwise
    if d <= R
        w = exp(-d^2 / (R^2 - d^2 + 1e-6));
    else
        w = 0;
    end
end

% --- Build Adjacency Matrix ---
function [Aconn, n_edges_total, n_edges_filtered] = build_adjacency_matrix(...
        N, positions_estimated, position_confidences, known_robots, ...
        direct_neigh, i, R_glob, edge_conf_threshold)
    % Build adjacency matrix with phantom edge filtering
    
    Aconn = zeros(N);
    n_edges_total = 0;
    n_edges_filtered = 0;
    
    for j1 = 1:N-1
        for j2 = j1+1:N
            % Skip low-confidence edges
            if position_confidences(j1) < edge_conf_threshold || ...
               position_confidences(j2) < edge_conf_threshold
                continue;
            end
            
            % Phantom edge filter: only include edges between known robots
            j1_valid = (j1 == i) || ismember(j1, direct_neigh) || ismember(j1, known_robots);
            j2_valid = (j2 == i) || ismember(j2, direct_neigh) || ismember(j2, known_robots);
            
            if ~j1_valid || ~j2_valid
                continue;
            end
            
            % Check distance
            dist_est = norm(positions_estimated(j1,:) - positions_estimated(j2,:));
            if dist_est <= R_glob
                n_edges_total = n_edges_total + 1;
                edge_weight = incmat_com(dist_est, R_glob);
                Aconn(j1, j2) = edge_weight;
                Aconn(j2, j1) = edge_weight;
                n_edges_filtered = n_edges_filtered + 1;
            end
        end
    end
end

% --- Find Connected Component ---
function component_robots = find_connected_component(Aconn, N, robot_id)
    % BFS to find connected component containing robot_id
    
    visited = false(N, 1);
    queue = robot_id;
    visited(robot_id) = true;
    component_robots = robot_id;
    
    while ~isempty(queue)
        current = queue(1);
        queue(1) = [];
        
        for neighbor = 1:N
            if Aconn(current, neighbor) > 0 && ~visited(neighbor)
                visited(neighbor) = true;
                queue = [queue, neighbor];
                component_robots = [component_robots, neighbor];
            end
        end
    end
end

% --- Compute Position Estimates ---
function [positions_est, velocities_est, confidences, stds] = ...
        estimate_robot_positions(N, i, x, v, direct_neigh, position_knowledge, ...
        t, x_start, t_start, x_goal, R_glob, CONFIG)
    % Estimate positions of all robots with confidence and uncertainty
    
    positions_est = zeros(N, 2);
    velocities_est = zeros(N, 2);
    confidences = zeros(N, 1);
    stds = zeros(N, 2);
    
    known_robots = [position_knowledge{i}.robot_id];
    
    for j = 1:N
        % --- Self (perfect knowledge) ---
        if j == i
            positions_est(j, :) = x(i, :);
            velocities_est(j, :) = v(i, :);
            confidences(j) = 100;
            stds(j, :) = [0.01, 0.01];
            
        % --- Direct neighbors (perfect knowledge) ---
        elseif ismember(j, direct_neigh)
            positions_est(j, :) = x(j, :);
            velocities_est(j, :) = v(j, :);
            confidences(j) = 100;
            stds(j, :) = [0.01, 0.01];
            
        % --- Remote robots (communication or blending) ---
        else
            comm_idx = find(known_robots == j, 1);
            has_valid_comm = false;
            
            % Try communication-based estimate
            if ~isempty(comm_idx)
                comm_data = position_knowledge{i}(comm_idx);
                comm_age = t - comm_data.timestamp;
                
                if comm_age < CONFIG.position_timeout
                    [positions_est(j,:), velocities_est(j,:), confidences(j), stds(j,:)] = ...
                        extrapolate_position(comm_data, comm_age, R_glob, CONFIG);
                    has_valid_comm = true;
                end
            end
            
            % Fallback to blended estimate
            if ~has_valid_comm
                [positions_est(j,:), velocities_est(j,:), confidences(j), stds(j,:)] = ...
                    blend_estimate(j, x_start, x_goal, x, i, t, t_start, R_glob, CONFIG);
            end
        end
    end
end

% --- Extrapolate Position with Velocity ---
function [pos_est, vel_est, conf, std_dev] = ...
        extrapolate_position(comm_data, comm_age, R_glob, CONFIG)
    % Extrapolate position using velocity and compute confidence
    
    if CONFIG.extrapolation_enabled && comm_age < CONFIG.extrapolation_max_age
        % Extrapolate position
        pos_est = comm_data.position + comm_data.velocity * comm_age;
        vel_est = comm_data.velocity;
        
        % Normalized age for dimensionless decay
        v_char = norm(comm_data.velocity);
        if v_char > 0.1
            normalized_age = (v_char * comm_age) / R_glob;
        else
            normalized_age = comm_age / CONFIG.extrapolation_max_age;
        end
        
        % Confidence decays with age and hops
        age_factor = exp(-2 * normalized_age);
        hop_factor = exp(-CONFIG.comm_decay_rate * comm_data.hop_count);
        conf = 90 * age_factor * hop_factor;
        
        % Uncertainty grows with extrapolation time
        std_scalar = 0.1 + 0.3 * normalized_age;
        std_dev = [std_scalar, std_scalar];
    else
        % No extrapolation, just use stale position
        pos_est = comm_data.position;
        vel_est = comm_data.velocity;
        
        age_factor = 1 - (comm_age / CONFIG.position_timeout);
        hop_factor = exp(-CONFIG.comm_decay_rate * comm_data.hop_count);
        conf = 90 * age_factor * hop_factor;
        std_dev = [0.2, 0.2];
    end
end

% --- Blend Starting Position with Goal/Random ---
function [pos_est, vel_est, conf, std_dev] = ...
        blend_estimate(robot_j, x_start, x_goal, x, robot_i, t, t_start, R_glob, CONFIG)
    % Blend starting position with goal (or random search) when no comm available
    
    SP_pos = x_start(robot_j, :);
    
    % Starting position confidence (decays over time)
    time_elapsed = t - t_start;
    normalized_time = time_elapsed / CONFIG.sp_decay_timescale;
    SP_conf = CONFIG.sp_conf_max * exp(-normalized_time) + CONFIG.sp_conf_min;
    
    if CONFIG.use_goal_blend
        % Blend with goal position (goal confidence grows as SP decays)
        Goal_pos = x_goal(robot_j, :);
        Goal_conf = (CONFIG.sp_conf_max + CONFIG.sp_conf_min) - SP_conf;
        
        pos_est = (SP_conf * SP_pos + Goal_conf * Goal_pos) / (SP_conf + Goal_conf);
        conf = (SP_conf + Goal_conf) / 2;
        
        std_scalar = 0.1 + 0.3 * normalized_time;
    else
        % Blend with random search
        est_dist = norm(x(robot_i,:) - x_start(robot_j,:));
        normalized_dist = max(0, (est_dist - R_glob) / (CONFIG.rs_decay_distance * R_glob));
        RS_conf = CONFIG.rs_conf_max * exp(-normalized_dist) + CONFIG.rs_conf_min;
        
        noise_scale = 0.1 + 0.5 * normalized_dist;
        RS_pos = x_start(robot_j,:) + noise_scale * randn(1,2);
        
        pos_est = (SP_conf * SP_pos + RS_conf * RS_pos) / (SP_conf + RS_conf);
        conf = (SP_conf + RS_conf) / 2;
        std_scalar = noise_scale;
    end
    
    vel_est = [0, 0];
    std_dev = [std_scalar, std_scalar];
end

% --- Monte Carlo λ2 Sampling ---
function [lambda2_mean, lambda2_std] = monte_carlo_lambda2(...
        N, robot_i, positions_estimated, position_confidences, position_std, ...
        R_glob, edge_conf_threshold, mc_samples)
    % Sample λ2 from uncertainty distribution
    
    lambda2_samples = zeros(mc_samples, 1);
    
    for mc = 1:mc_samples
        % Sample positions from Gaussian distributions
        positions_sampled = zeros(N, 2);
        for j = 1:N
            if position_confidences(j) >= edge_conf_threshold
                noise = position_std(j, :) .* randn(1, 2);
                positions_sampled(j, :) = positions_estimated(j, :) + noise;
            else
                positions_sampled(j, :) = positions_estimated(j, :);
            end
        end
        
        % Build adjacency for sampled positions
        Aconn_sample = zeros(N);
        for j1 = 1:N-1
            for j2 = j1+1:N
                if position_confidences(j1) >= edge_conf_threshold && ...
                   position_confidences(j2) >= edge_conf_threshold
                    dist_sample = norm(positions_sampled(j1,:) - positions_sampled(j2,:));
                    if dist_sample <= R_glob
                        edge_weight = incmat_com(dist_sample, R_glob);
                        Aconn_sample(j1, j2) = edge_weight;
                        Aconn_sample(j2, j1) = edge_weight;
                    end
                end
            end
        end
        
        % Find component and compute λ2
        component_robots = find_connected_component(Aconn_sample, N, robot_i);
        
        if length(component_robots) >= 2
            A_comp = Aconn_sample(component_robots, component_robots);
            L_comp = diag(sum(A_comp, 2)) - A_comp;
            ev_comp = sort(eig(L_comp));
            lambda2_samples(mc) = ev_comp(2);
        else
            lambda2_samples(mc) = 0;
        end
    end
    
    lambda2_mean = mean(lambda2_samples);
    lambda2_std = std(lambda2_samples);
end

% --- Validation Checks ---
function validation_flag = validate_lambda2(...
        lambda2_est, robot_i, n_component, local_degree, Aconn_local, ...
        component_robots, CONFIG)
    % Perform validation heuristics on λ2 estimate
    
    validation_flag = 0;
    
    % Check 1: λ2 should be ≤ n-1
    if lambda2_est > n_component - 1 + 0.01
        validation_flag = 1;
        if CONFIG.debug_estimation
            fprintf('⚠️  Robot %d: λ2=%.4f > n-1=%d (INVALID)\n', ...
                    robot_i, lambda2_est, n_component-1);
        end
        return;
    end
    
    % Check 2: λ2≈0 but has neighbors (suspicious)
    if lambda2_est < 0.001 && local_degree > 0
        validation_flag = 2;
        if CONFIG.debug_estimation
            fprintf('⚠️  Robot %d: λ2≈0 but degree=%d (SUSPICIOUS)\n', ...
                    robot_i, local_degree);
        end
        return;
    end
    
    % Check 3: Weak spectral gap
    if length(component_robots) >= 3
        A_comp = Aconn_local(component_robots, component_robots);
        L_comp = diag(sum(A_comp, 2)) - A_comp;
        ev_comp = sort(eig(L_comp));
        if lambda2_est > 0.8 * ev_comp(3)
            validation_flag = 3;
        end
    end
end

% --- Robust Consensus ---
function lambda2_refined = robust_consensus(...
        robot_i, lambda2_own, lambda2_estimates, lambda2_confidence, x, R_glob, CONFIG)
    % Refine λ2 estimate using weighted consensus with outlier rejection
    
    dists = sqrt(sum((x - x(robot_i,:)).^2, 2));
    neigh = find(dists <= R_glob & (1:length(x))' ~= robot_i);
    
    if length(neigh) < CONFIG.min_consensus_neighbors
        lambda2_refined = lambda2_own;
        return;
    end
    
    % Collect neighbor estimates with weights
    neighbor_estimates = lambda2_estimates(neigh);
    neighbor_confidences = lambda2_confidence(neigh);
    neighbor_degrees = arrayfun(@(j) sum(sqrt(sum((x - x(j,:)).^2, 2)) <= R_glob), neigh);
    
    % Weight by confidence × degree
    weights = neighbor_confidences .* neighbor_degrees';
    weights = weights / sum(weights);
    
    % Outlier rejection
    local_mean = sum(neighbor_estimates .* weights);
    local_std = sqrt(sum(weights .* (neighbor_estimates - local_mean).^2));
    valid_mask = abs(neighbor_estimates - local_mean) <= CONFIG.outlier_threshold * local_std;
    
    if sum(valid_mask) >= CONFIG.min_consensus_neighbors
        filtered_estimates = neighbor_estimates(valid_mask);
        filtered_weights = weights(valid_mask);
        filtered_weights = filtered_weights / sum(filtered_weights);
        
        consensus_estimate = sum(filtered_estimates .* filtered_weights);
        lambda2_refined = 0.7 * lambda2_own + 0.3 * consensus_estimate;
    else
        lambda2_refined = lambda2_own;
    end
end

% --- Debug Output ---
function print_estimation_debug(robot_i, t, n_direct, n_comm, n_blend, ...
        n_edges_filtered, n_edges_total, n_component, N, local_deg, ...
        Aconn_local, component_robots, lambda2_est, lambda2_std_val, ...
        lambda2_conf, lambda2_cons, CONFIG, validation_flag)
    % Compact debug output for estimation process
    
    fprintf('[R%d @ t=%.2fs] ', robot_i, t);
    fprintf('Pos:%dD/%dC/%dB | ', n_direct, n_comm, n_blend);
    fprintf('Edges:%d/%d | ', n_edges_filtered, n_edges_total);
    
    % Component info
    if n_component >= 2
        A_comp = Aconn_local(component_robots, component_robots);
        dens = 100*sum(A_comp(:)>0)/(n_component*(n_component-1));
        fprintf('Comp:%d/%d(d=%d,ρ=%.0f%%) | ', n_component, N, local_deg, dens);
    else
        fprintf('Comp:%d/%d | ', n_component, N);
    end
    
    % λ2 estimate
    if CONFIG.mc_enabled
        cv = lambda2_std_val/(lambda2_est+1e-6);
        fprintf('λ2=%.4f±%.4f(CV=%.1f%%) | ', lambda2_est, lambda2_std_val, 100*cv);
    else
        fprintf('λ2=%.4f±%.4f | ', lambda2_est, lambda2_std_val);
    end
    
    % Confidence
    fprintf('Conf=%.1f%%', lambda2_conf);
    
    % Consensus
    if CONFIG.consensus_enabled && abs(lambda2_cons - lambda2_est) > 1e-6
        fprintf(' (cons=%.4f)', lambda2_cons);
    end
    
    % Warnings
    if validation_flag > 0
        fprintf(' | ⚠️');
    end
    
    fprintf('\n');
end


%% ========================================================================
%%  3. INITIALIZATION
%% ========================================================================

fig = figure;
x_start = x;
t_start = 0;

% Enhanced message structure: position, velocity, timestamp
for i = 1:N
    position_knowledge{i} = struct('robot_id', {}, 'position', {}, 'velocity', {}, ...
                                   'timestamp', {}, 'hop_count', {});
    position_knowledge{i}(1) = struct('robot_id', i, 'position', x(i,:), ...
                                      'velocity', v(i,:), 'timestamp', 0, 'hop_count', 0);
end

colors = hsv(N);

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

% Calculate axis limits
all_x = [x(:,1); x_goal(:,1); obs_pos(:,1)];
all_y = [x(:,2); x_goal(:,2); obs_pos(:,2)];
margin = 2.0;
axis_xlim = [min(all_x) - margin, max(all_x) + margin];
axis_ylim = [min(all_y) - margin, max(all_y) + margin];


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
        
        % --- Compute λ2 estimate ---
        if CONFIG.mc_enabled && n_component >= 2
            % Monte Carlo sampling
            [lambda2_estimates(i), lambda2_std(i)] = monte_carlo_lambda2(...
                N, i, positions_estimated, position_confidences, position_std, ...
                R_glob, CONFIG.edge_conf_threshold, CONFIG.mc_samples);
            
            % Confidence based on position quality and MC consistency
            avg_conf = mean(position_confidences(component_robots));
            mc_cv = lambda2_std(i) / (lambda2_estimates(i) + 1e-6);
            mc_confidence = exp(-2 * mc_cv);
            lambda2_confidence(i) = avg_conf * mc_confidence * (n_component / N);
            
        else
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
        end
        
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
    
    
    %% ====================================================================
    %%  4.6 VISUALIZATION
    %% ====================================================================
    
    if mod(k,5)==0
        clf;
        hold on;
        if ~isvalid(fig) || ~ishandle(fig)
            disp('Figure closed.');
            return;
        end
        
        % Build actual adjacency
        Aconn = zeros(N);
        for i=1:N-1
            for j=i+1:N
                dist = norm(x(i,:)-x(j,:));
                if dist <= R_glob
                    Aconn(i,j) = incmat_com(dist, R_glob);
                    Aconn(j,i) = Aconn(i,j);
                end
            end
        end
        
        % Draw obstacles
        for o=1:nObs
            viscircles(obs_pos(o,:),obs_rad(o),'Color','r');
            viscircles(obs_pos(o,:),obs_rad(o)+rsafe,'Color',[1 0.7 0.7],'LineStyle',':');
        end
        
        % Draw connectivity edges
        for i=1:N-1
            for j=i+1:N
                if Aconn(i,j)>0
                    plot([x(i,1),x(j,1)],[x(i,2),x(j,2)],'k-','LineWidth',0.5);
                end
            end
        end
        
        % Draw robots
        for i=1:N
            % Color by uncertainty
            if lambda2_std(i) > 0.2
                marker_color = [1, 0.5, 0];  % Orange for high uncertainty
            else
                marker_color = colors(i,:);
            end
            
            scatter(x(i,1), x(i,2), 80, marker_color, 'filled');
            quiver(x(i,1), x(i,2), v(i,1), v(i,2), 0.4, 'Color', colors(i,:), 'LineWidth', 1.5);
            scatter(x_goal(i,1), x_goal(i,2), 100, colors(i,:), 'x', 'LineWidth', 2);
            plot([x(i,1), x_goal(i,1)], [x(i,2), x_goal(i,2)], ':', 'Color', colors(i,:), 'LineWidth', 0.5);
            
            % Label with warning if needed
            if validation_flags(k, i) > 0
                label_str = sprintf('R%d⚠️', i);
            else
                label_str = sprintf('R%d', i);
            end
            text(x(i,1), x(i,2)+0.2, label_str, 'FontSize', 8, 'HorizontalAlignment', 'center');
        end
        
        axis equal; grid on;
        xlim(axis_xlim); ylim(axis_ylim);
        xlabel('x [m]'); ylabel('y [m]');
        title(sprintf('Comprehensive CBF | t=%.2f | λ2=%.3f±%.3f (conf=%.0f%%)', ...
                      t, lambda2_avg, lambda2_std_avg, lambda2_conf_avg));
        drawnow;
    end
end


%% ========================================================================
%%  5. FINAL REPORT
%% ========================================================================

fprintf('\n╔═══════════════════════════════════════════════════════════════╗\n');
fprintf('║        COMPREHENSIVE SIMULATION COMPLETE                      ║\n');
fprintf('╚═══════════════════════════════════════════════════════════════╝\n\n');

fprintf('FINAL λ2 ESTIMATES:\n');
fprintf('─────────────────────────────────────────────────────────────\n');
for i = 1:N
    fprintf('  Robot %d: λ2=%.4f±%.4f (conf=%.1f%%, consensus=%.4f)\n', ...
            i, lambda2_estimates(i), lambda2_std(i), lambda2_confidence(i), lambda2_consensus(i));
end

fprintf('\nVALIDATION SUMMARY:\n');
fprintf('  Total warnings: %d\n', sum(validation_flags(:) > 0));
fprintf('  λ2 > n-1 violations: %d\n', sum(validation_flags(:) == 1));
fprintf('  Suspicious disconnections: %d\n', sum(validation_flags(:) == 2));
fprintf('  Weak spectral gaps: %d\n', sum(validation_flags(:) == 3));

fprintf('\nλ2 TRAJECTORY:\n');
fprintf('  Mean: %.4f ± %.4f\n', mean(lambda2_log), mean(lambda2_std_log));
fprintf('  Final: %.4f ± %.4f (conf=%.1f%%)\n', lambda2_avg, lambda2_std_avg, lambda2_conf_avg);

fprintf('\nFEATURES ENABLED:\n');
if CONFIG.extrapolation_enabled
    fprintf('  ✓ Velocity extrapolation\n');
end
if CONFIG.mc_enabled
    fprintf('  ✓ Monte Carlo uncertainty (%d samples)\n', CONFIG.mc_samples);
end
if CONFIG.consensus_enabled
    fprintf('  ✓ Robust consensus (outlier rejection)\n');
end
if CONFIG.validation_enabled
    fprintf('  ✓ Validation heuristics\n');
end
fprintf('  ✓ Normalized decay parameters\n');
fprintf('  ✓ Phantom edge filtering\n');

fprintf('\n╚═══════════════════════════════════════════════════════════════╝\n');
disp("✓ Comprehensive version complete!")