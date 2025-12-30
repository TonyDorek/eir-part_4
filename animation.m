%% ANIMATION OF ROBOT DYNAMICS

fig = figure;

if opt_strategy ~= "Distributed" % Re-shaping x and v tensors in output from Simulink
    xlog = reshape_log(xlog_sim);
    vlog = reshape_log(vlog_sim);
end

% Calculating axis limits
all_x = [xlog(1,:,1)'; x_goal(:,1); obs_pos(:,1)];
all_y = [xlog(1,:,2)'; x_goal(:,2); obs_pos(:,2)];
margin = 1.0;
axis_xlim = [min(all_x) - margin, max(all_x) + margin];
axis_ylim = [min(all_y) - margin, max(all_y) + margin];

% Finding unique rows and check for duplicates - Used for goal plotting
[uniqueRows, ~] = unique(x_goal, 'rows');
hasDuplicateRows = size(uniqueRows, 1) < size(x_goal, 1);

l = size(xlog);

disp("Simulation started!")

for k = 1:l(1)
    if mod(k,5)==0

        clf; hold on; grid on; axis equal;
        if ~isvalid(fig) || ~ishandle(fig)
            disp('Figure closed.');
            close;
            return;
        end

        XK = squeeze(xlog(k,:,:)); % Nx2
        VK = squeeze(vlog(k,:,:)); % Nx2

        % Animation setup
        xlim(axis_xlim);
        ylim(axis_ylim);
        xlabel('x [m]'); ylabel('y [m]');
        if opt_strategy ~= "Distributed"
            title(sprintf('Optimization => %s | Time = %.2f s', opt_strategy, (k-1)*dt));
        else
            title(sprintf('Optimization => %s | Time = %.2f | λ2=%.3f±%.3f (conf=%.0f%%)', ...
            opt_strategy, (k-1)*dt, lambda2_log(k), lambda2_std_log(k), lambda2_conf_log(k)));
        end

        % Drawing obstacles
        for o=1:nObs
            viscircles(obs_pos(o,:),obs_rad(o),'Color','r');
            viscircles(obs_pos(o,:),obs_rad(o)+rsafe,'Color',[1 0.7 0.7],'LineStyle',':');
        end
    
        % Drawing connections between (if dist <= R_glob)    
        for i = 1:N-1
            for j = i+1:N
                dist = norm(XK(i,:) - XK(j,:));
                if dist <= R_glob
                    plot([XK(i,1), XK(j,1)], ...
                         [XK(i,2), XK(j,2)], 'k-', 'LineWidth', 0.5);
                end
            end
        end
  
        % Drawing robots
        for i=1:N
            marker_color = colors(i,:);
            scatter(XK(i,1), XK(i,2), 100, colors(i,:), 'filled');
            quiver(XK(i,1), XK(i,2), VK(i,1), VK(i,2), 0.4, 'Color', colors(i,:), 'LineWidth', 1.5);
            if hasDuplicateRows
                scatter(x_goal(i,1), x_goal(i,2), 100, 'black', 'x', 'LineWidth', 2);
            else
                scatter(x_goal(i,1), x_goal(i,2), 100, colors(i,:), 'x', 'LineWidth', 2);
            end
            plot([XK(i,1), x_goal(i,1)], [XK(i,2), x_goal(i,2)], ':', 'Color', colors(i,:), 'LineWidth', 0.5)
            label_str = sprintf('R%d', i);
            text(XK(i,1), XK(i,2)+0.3, label_str, 'FontSize', 10, 'HorizontalAlignment', 'center');
        end
        drawnow;
        % pause(0.03);
    end
end

if opt_strategy == "Distributed"
    fprintf('\n╔═══════════════════════════════════════════════════════════════╗\n');
    fprintf('║        DISTRIBUTED λ2 COMPUTATION                             ║\n');
    fprintf('╚═══════════════════════════════════════════════════════════════╝\n\n');   
    fprintf('FINAL λ2 ESTIMATES:\n');
    fprintf('─────────────────────────────────────────────────────────────\n');
    for i = 1:N
        fprintf('  Robot %d: λ2=%.4f±%.4f (conf=%.1f%%, consensus=%.4f)\n', ...
                i, lambda2_estimates(i), lambda2_std(i), lambda2_confidence(i), lambda2_consensus(i));
    end
    
    % fprintf('\nVALIDATION SUMMARY:\n');
    % fprintf('  Total warnings: %d\n', sum(validation_flags(:) > 0));
    % fprintf('  λ2 > n-1 violations: %d\n', sum(validation_flags(:) == 1));
    % fprintf('  Suspicious disconnections: %d\n', sum(validation_flags(:) == 2));
    % fprintf('  Weak spectral gaps: %d\n', sum(validation_flags(:) == 3));
    
    fprintf('\nλ2 TRAJECTORY:\n');
    fprintf('  Mean over time: %.4f ± %.4f\n', mean(lambda2_log), mean(lambda2_std_log));
    fprintf('  Final: %.4f ± %.4f (conf=%.1f%%)\n', lambda2_avg, lambda2_std_avg, lambda2_conf_avg);
    
    fprintf('\nFEATURES ENABLED:\n');
    if CONFIG.extrapolation_enabled
        fprintf('  ✓ Velocity extrapolation\n');
    end
    if CONFIG.consensus_enabled
        fprintf('  ✓ Robust consensus (outlier rejection)\n');
    end
    if CONFIG.validation_enabled
        fprintf('  ✓ Validation heuristics\n');
    end
    fprintf('  ✓ Normalized decay parameters\n');
    fprintf('  ✓ Phantom edge filtering\n');
end    
%    fprintf('\n╚═══════════════════════════════════════════════════════════════╝\n');
    disp("✓ Simulation completed!")
%end