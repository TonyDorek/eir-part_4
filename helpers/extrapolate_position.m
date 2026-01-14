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
        % AD - The higher is the normalized_age, the less is the confidence
        % about robot position estimate (too fast velocity or too old info)
        hop_factor = exp(-CONFIG.comm_decay_rate * comm_data.hop_count);
        % AD - The higher is the hop count, the less is the confidence
        % about robot position estimate (too many passages from one robot to another)      
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