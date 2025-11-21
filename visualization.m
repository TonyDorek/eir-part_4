%% VISUALIZATION SCRIPT
% Run it AFTER executing the main script...

%% --- Plot state evolution ---
figure;
set(gcf, 'Position', [200, 100, 1200, 400]); % [x, y, width, height]

subplot(1, 2, 1);
plot((0:k-1)*dt,xlog(1:k,:,1),'LineWidth',1.4);
xlabel('time [s]'); ylabel('x [m]'); grid minor;% yticks(-R0:0.5:R0);
title(sprintf('%d-agents system - Position x evolution', N));

subplot(1, 2, 2);
plot((0:k-1)*dt,xlog(1:k,:,2),'LineWidth',1.4);
xlabel('time [s]'); ylabel('y [m]'); grid minor;% yticks(-R0:0.5:R0);
title(sprintf('%d-agents system - Position y evolution', N));

%% --- Plot connectivity parameters ---
if opt_strategy == 'hybrid'
    figure;
    set(gcf, 'Position', [200, 600, 1200, 400])
    subplot(1,2,1);
    plot((0:k-1)*dt, lambda2_log(1:k), 'LineWidth', 1.4);
    hold on; yline(lambda2_eps, 'r--', 'λ2_{eps}');
    yline(lambda2_warn, 'g--', 'λ2_{warn}');
    xlabel('time [s]'); ylabel('\lambda_2'); grid on;
    title('\lambda_2 (connectivity) evolution');
    
    subplot(1,2,2);
    plot((0:k-1)*dt, gamma_log(1:k), 'LineWidth', 1.4);
    xlabel('time [s]'); ylabel('\gamma_{glob}'); grid on;
    title('\gamma_{glob} (gain factor) evolution');

    figure;
    set(gcf, 'Position', [500, 300, 600, 400])
    bar([d_main, d_out]);
    ylabel('Distance [m]');
    xticklabels({'Main group', 'Outlier'});
    title('Task Accuracy (respective goals)');
    grid on;

else
    figure;
    plot((0:k-1)*dt,lambda2_log(1:k),'LineWidth',1.4);
    xlabel('time [s]'); ylabel('\lambda_2'); grid on;
    title('\lambda_2 (connectivity)');
end