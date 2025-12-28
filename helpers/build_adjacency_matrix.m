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