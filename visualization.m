%% ANIMATION OF ROBOT DYNAMICS
% Run it AFTER executing the Simulink model...

fig = figure;

% --- Re-shaping x and v tensors in output from Simulink ---
xlog = reshape_log(xlog_sim);
vlog = reshape_log(vlog_sim);

% --- Building time vector ---
l = size(xlog);
M = l(1);
t = (0:M-1)' * dt;   % [0, dt, 2dt, ..., (M-1)dt]

% --- Re-shaping (or computing) lambda ---
if opt_strategy == "Distributed"
    lambdaslog = reshape_log(lambdas);
else
    lambda2_log = zeros(M,1);
    for k = 1:M
        Xk = squeeze(xlog(k,:,:));   % N x 2, position of all the agents at time k
    
        % Adjacency matrix based om distance and R_glob
        Aconn = zeros(N);
        for i = 1:N-1
            for j = i+1:N
                dist = norm2(Xk(i,:) - Xk(j,:));
                if dist <= R_glob
                    Aconn(i,j) = incmat_com(dist,R_glob);
                    Aconn(j,i) = Aconn(i,j);
                end
            end
        end
    
        % Laplacian and eigenvalues
        L  = diag(sum(Aconn,2)) - Aconn;
        ev = sort(eig(L));
        if numel(ev) >= 2
            lambda2_log(k) = ev(2);
        else
            lambda2_log(k) = 0;
        end
    end 
end

% --- Calculating axis limits ---
all_x = [xlog(1,:,1)'; x_goal(:,1); obs_pos(:,1)];
all_y = [xlog(1,:,2)'; x_goal(:,2); obs_pos(:,2)];
margin = 1.0;
axis_xlim = [min(all_x) - margin, max(all_x) + margin];
axis_ylim = [min(all_y) - margin, max(all_y) + margin];

% --- Finding unique rows and check for duplicates (used for goal plotting) ---
[uniqueRows, ~] = unique(x_goal, 'rows');
hasDuplicateRows = size(uniqueRows, 1) < size(x_goal, 1);

% --- Start simulation ---
fprintf("Simulation started!")

for k = 1:M
    timestamp = (k-1)*dt;
    if mod(timestamp,0.25)==0

        clf; hold on; grid on; axis equal;
        if ~isvalid(fig) || ~ishandle(fig)
            fprintf('Animation closed. Visualization program interrupted :(\n');
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
            lambda_est = lambdaslog(:,:,1); % it's lambda2_estimates, [200,5]
            lambda_con = lambdaslog(:,:,2); % it's lambda2_consensus, [200,5]
            std = lambdaslog(:,:,3); % it's lambda2_std, [200,5]
            con = lambdaslog(:,:,4); % it's lambda2_confidence, [200,5]            
        
            lambda2_log = mean(lambda_con,2); % it's lambda2_avg, [200, 1]
            lambda2_std_log = mean(std,2); % it's lambda2_std_avg, [200,1]
            lambda2_conf_log = mean(con,2); % it's lambda2_conf_avg, [200,1]

            title(sprintf('Optimization => %s | Time = %.2f | λ2=%.3f±%.3f (conf=%.0f%%)', ...
            opt_strategy, timestamp, lambda2_log(k), lambda2_std_log(k), lambda2_conf_log(k)));
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
            plot(squeeze(xlog(1:k,i,1)), squeeze(xlog(1:k,i,2)), 'Color', colors(i,:), 'LineWidth', 1, 'LineStyle', '--');
            scatter(x_start(i,1), x_start(i,2), 100, colors(i,:), 'o', 'LineWidth', 2);
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
    end
end

if opt_strategy == "Distributed"
    fprintf('\n╔═══════════════════════════════════════════════════════════════╗\n');
    fprintf('║        DISTRIBUTED λ2 COMPUTATION                             ║\n');
    fprintf('╚═══════════════════════════════════════════════════════════════╝\n\n');   
    fprintf('FINAL λ2 ESTIMATES:\n');
    fprintf('─────────────────────────────────────────────────────────────\n')

    for i = 1:N
        fprintf('  Robot %d: λ2=%.4f±%.4f (conf=%.1f%%, consensus=%.4f)\n', ...
            i, lambda_est(end,i), std(end,i), con(end,i), lambda_con(end,i));
    end
    
    fprintf('\nλ2 TRAJECTORY:\n');
    fprintf('  Mean over time: %.4f ± %.4f\n', mean(lambda2_log), mean(lambda2_std_log));
    fprintf('  Final: %.4f ± %.4f (conf=%.1f%%)\n',  lambda2_log(end), lambda2_std_log(end), lambda2_conf_log(end));
    
    fprintf('\nFEATURES ENABLED:\n');
    if CONFIG.extrapolation_enabled
        fprintf('  ✓ Velocity extrapolation\n');
    end
    if CONFIG.consensus_enabled
        fprintf('  ✓ Robust consensus (outlier rejection)\n');
    end
end    

fprintf('\n✓ Simulation completed!!\n')

%% PLOT VISUALIZATION

% --- Plotting the position evolution ---
figure;
set(gcf, 'Position', [200, 100, 1200, 400]); % [x, y, width, height]

subplot(1, 2, 1);
plot(t, xlog(:,:,1),'LineWidth',1.4);  % tutte le x degli N agenti
xlabel('time [s]'); ylabel('x [m]'); grid minor;
title(sprintf('%d-agents system - Position x evolution', N));
legend('R1','R2','R3','R4','R5')

subplot(1, 2, 2);
plot(t, xlog(:,:,2),'LineWidth',1.4);  % tutte le y degli N agenti
xlabel('time [s]'); ylabel('y [m]'); grid minor;
title(sprintf('%d-agents system - Position y evolution', N));
legend('R1','R2','R3','R4','R5')

% --- Plotting the velocity evolution ---
figure;
set(gcf, 'Position', [200, 600, 1200, 400]);

subplot(1, 2, 1);
plot(t, vlog(:,:,1),'LineWidth',1.4);  % tutte le v_x degli N agenti
xlabel('time [s]'); ylabel('v_x [m/s]'); grid minor;
title(sprintf('%d-agents system - Velocity v_x evolution', N));
legend('R1','R2','R3','R4','R5')

subplot(1, 2, 2);
plot(t, vlog(:,:,2),'LineWidth',1.4);  % tutte le v_y degli N agenti
xlabel('time [s]'); ylabel('v_y [m/s]'); grid minor;
title(sprintf('%d-agents system - Velocity v_y evolution', N));
legend('R1','R2','R3','R4','R5')

% --- Plotting the global connectivity evolution ---
figure;
set(gcf, 'Position', [500, 600, 600, 400])
plot(t, lambda2_log,'LineWidth',1.4);
xlabel('time [s]'); ylabel('\lambda_2'); grid on;
title('\lambda_2 (connectivity)');

if opt_strategy == "Distributed"
    figure;
    set(gcf, 'Position', [200, 100, 1200, 400]);
    
    subplot(1, 2, 1);
    plot(t, lambda_est,'LineWidth',1.4);
    xlabel('time [s]'); ylabel('\lambda_2'); grid on;
    title('\lambda_2 estimation per robot (no consensus)');
    legend('R1','R2','R3','R4','R5')
    
    subplot(1, 2, 2);
    plot(t, lambda_con,'LineWidth',1.4);
    xlabel('time [s]'); ylabel('y [m]'); grid on;
    title('\lambda_2 estimation per robot (with consensus)');
    legend('R1','R2','R3','R4','R5')
end