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
        % AD - The more the robot is far from the start position (so high
        % normalized_dist) the more is the noise on the attempt to
        % estimate its position 
        
        pos_est = (SP_conf * SP_pos + RS_conf * RS_pos) / (SP_conf + RS_conf);
        conf = (SP_conf + RS_conf) / 2;
        std_scalar = noise_scale;
    end
    
    vel_est = [0, 0];
    std_dev = [std_scalar, std_scalar];
end