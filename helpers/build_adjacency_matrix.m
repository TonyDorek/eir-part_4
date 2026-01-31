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
            
            % DOES THIS SPECIFIC CASE INVOLVE THE EGO ROBOT?
            if (j1 == i || j2 == i) % EGO-ADJACENT EDGE
                
                
                % Identify the "other" robot in this pair
                if j1 == i
                    other = j2;
                else
                    other = j1;
                end
                
                % IF the other robot is not a direct neighbor we skip this edge
                % - if it's a direct neighbor, we know it's connected
                % - if it's not a direct neighbor, we know it's NOT connected (estimates could put it within range
                if ~ismember(other, direct_neigh)
                    continue; 
                end
                
                dist_val = norm(positions_estimated(j1,:) - positions_estimated(j2,:)); %IT IS GROUND TRUTH
                
            else                    % REMOTE-REMOTE EDGE
                
                % CONFIDENCE THRESHOLD CHECK
                if position_confidences(j1) < edge_conf_threshold || ...
                   position_confidences(j2) < edge_conf_threshold
                    continue;
                end
                
                % UNKNOWN ROBOT CHECK
                if ~ismember(j1, known_robots) || ~ismember(j2, known_robots)
                    continue;
                end
                
                % DISTANCE COMPUTATION
                dist_val = norm(positions_estimated(j1,:) - positions_estimated(j2,:));
            end
            
            %Final distance check to determine connectivity
            if dist_val <= R_glob
                n_edges_total = n_edges_total + 1;
                edge_weight = incmat_com(dist_val, R_glob);
                Aconn(j1, j2) = edge_weight;
                Aconn(j2, j1) = edge_weight;
                n_edges_filtered = n_edges_filtered + 1;
            end
        end
    end
end