% --- Validation Checks ---
function validation_flag = validate_lambda2(...
        lambda2_est, robot_i, n_component, local_degree, Aconn_local, ...
        component_robots, CONFIG)
    % Perform validation heuristics on λ2 estimate
    
    validation_flag = 0;
    
    % Check 1: λ2 should be ≤ n-1
    % if lambda2_est > n_component - 1 + 0.01
    %     validation_flag = 1;
    %     if CONFIG.debug_estimation
    %         fprintf('⚠️  Robot %d: λ2=%.4f > n-1=%d (INVALID)\n', ...
    %                 robot_i, lambda2_est, n_component-1);
    %     end
    %     return;
    % end
    
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
    % if length(component_robots) >= 3
    %     A_comp = Aconn_local(component_robots, component_robots);
    %     L_comp = diag(sum(A_comp, 2)) - A_comp;
    %     ev_comp = sort(eig(L_comp));
    %     if lambda2_est > 0.8 * ev_comp(3)
    %         validation_flag = 3;
    %     end
    % end
end
