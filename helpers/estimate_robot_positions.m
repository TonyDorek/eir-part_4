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