%% VISUALIZATION SCRIPT (Simulink version)
% Run it AFTER executing the Simulink model, e.g.:

%% --- Capire la dimensione di xlog_sim e ricostruire xlog ---
% xlog(k,i,j): k = tempo, i = agente, j = coordinata (1=x, 2=y)
if exist('xlog', 'var') == 0
    xlog = reshape_log(xlog_sim); % Re-shaping x tensor in output from Simulink
end
%% --- Costruzione del vettore tempo ---
l = size(xlog);
M = l(1);
t = (0:M-1)' * dt;   % [0, dt, 2dt, ..., (M-1)dt]

%% --- Plot evoluzione delle posizioni ---
figure;
set(gcf, 'Position', [200, 100, 1200, 400]); % [x, y, width, height]

subplot(1, 2, 1);
plot(t, xlog(:,:,1),'LineWidth',1.4);  % tutte le x degli N agenti
xlabel('time [s]'); ylabel('x [m]'); grid minor;
title(sprintf('%d-agents system - Position x evolution', N));

subplot(1, 2, 2);
plot(t, xlog(:,:,2),'LineWidth',1.4);  % tutte le y degli N agenti
xlabel('time [s]'); ylabel('y [m]'); grid minor;
title(sprintf('%d-agents system - Position y evolution', N));

if opt_strategy ~= "Distributed"
    %% --- Calcolo lambda2_log (connettività globale) ---
    lambda2_log = zeros(M,1);
    for k = 1:M
        Xk = squeeze(xlog(k,:,:));   % N x 2, posizione di tutti gli agenti all'istante k
    
        % matrice di adiacenza basata sulla distanza e R_glob
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
    
        % Laplaciano e autovalori
        L  = diag(sum(Aconn,2)) - Aconn;
        ev = sort(eig(L));
        if numel(ev) >= 2
            lambda2_log(k) = ev(2);
        else
            lambda2_log(k) = 0;
        end
    end

    %% --- Plot della connettività λ2 ---
    if opt_strategy == "Hybrid"
        % Caso ibrido: se hai nel workspace lambda2_eps, lambda2_warn, gamma_log
        % lambda2_eps  = evalin('base','lambda2_eps');
        % lambda2_warn = evalin('base','lambda2_warn');
        % gamma_log    = evalin('base','gamma_log');
    
        figure;
        set(gcf, 'Position', [200, 600, 1200, 400])
    
        subplot(1,2,1);
        plot(t, lambda2_log, 'LineWidth', 1.4);
        hold on;
        yline(lambda2_eps,  'r--', 'λ2_{eps}');
        yline(lambda2_warn, 'g--', 'λ2_{warn}');
        xlabel('time [s]'); ylabel('\lambda_2'); grid on;
        title('\lambda_2 (connectivity) evolution');
    
        subplot(1,2,2);
        % se gamma_log è più corto di M, taglia; se è più lungo, usa solo i primi M
        m_gamma = min(length(gamma_log), M);
        for k = 1:m_gamma
            if lambda2_log(k) < lambda2_warn
                gamma_log(k) = 1 + k_lambda_glob * (lambda2_warn - lambda2_log(k));
                gamma_log(k) = min(gamma_log(k), gamma_max);
            else
                gamma_log(k) = 1.0;
            end
        end
        plot(t(1:m_gamma), gamma_log(1:m_gamma), 'LineWidth', 1.4);
        xlabel('time [s]'); ylabel('\gamma_{glob}'); grid on;
        title('\gamma_{glob} (gain factor) evolution');  
    end    
end

if opt_strategy ~= "Hybrid"
    figure;
    set(gcf, 'Position', [500, 500, 600, 400])
    plot(t, lambda2_log,'LineWidth',1.4);
    xlabel('time [s]'); ylabel('\lambda_2'); grid on;
    title('\lambda_2 (connectivity)');
end