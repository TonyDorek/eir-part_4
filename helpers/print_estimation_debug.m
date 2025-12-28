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
    % if CONFIG.mc_enabled
    %     cv = lambda2_std_val/(lambda2_est+1e-6);
    %     fprintf('λ2=%.4f±%.4f(CV=%.1f%%) | ', lambda2_est, lambda2_std_val, 100*cv);
    % else
        fprintf('λ2=%.4f±%.4f | ', lambda2_est, lambda2_std_val);
    % end
    
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