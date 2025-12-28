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