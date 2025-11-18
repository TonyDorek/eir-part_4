%% VISUALIZATION SCRIPT
% Run it AFTER executing the main script...

%% --- Plot state evolution ---------------------------------------------------
figure;
set(gcf, 'Position', [250, 100, 1200, 400]); % [x, y, width, height]

subplot(1, 2, 1);
plot((0:k-1)*dt,xlog(1:k,:,1),'LineWidth',1.4);
xlabel('time [s]'); ylabel('x [m]'); grid minor;% yticks(-R0:0.5:R0);
title(sprintf('%d-agents system - Position x evolution', N));

% figure;
subplot(1, 2, 2);
plot((0:k-1)*dt,xlog(1:k,:,2),'LineWidth',1.4);
xlabel('time [s]'); ylabel('y [m]'); grid minor;% yticks(-R0:0.5:R0);
title(sprintf('%d-agents system - Position y evolution', N));

%% --- Plot λ₂ ---------------------------------------------------
figure;
plot((0:k-1)*dt,lambda2_log(1:k),'LineWidth',1.4);
xlabel('time [s]'); ylabel('\lambda_2 (connectivity)'); grid on;
title('Connectivity evolution');