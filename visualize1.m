%% VISUALIZATION SCRIPT (Simulink version)
% Run it AFTER executing the Simulink model, e.g.:
%   initialize;
%   sim('simu');   % o sim('simlk'), sim('model'), ecc.
%
% Richiede che il modello abbia un blocco To Workspace collegato a x_vec
% che salvi la variabile "xlog_sim" in formato Array.

clc;

%% --- Controllo variabili necessarie ---
if ~exist('xlog_sim','var')
    error(['Variable xlog_sim not found in workspace. ' ...
           'Make sure your Simulink model has a To Workspace block ' ...
           'connected to x\_vec with name "xlog_sim".']);
end

% Parametri dal workspace base (definiti in initialize.m)
dt           = evalin('base','dt');
R_glob       = evalin('base','R_glob');
opt_strategy = evalin('base','opt_strategy');  % "centralized", "decentralized", "hybrid"
opt_str      = string(opt_strategy);

%% --- Capire la dimensione di xlog_sim e ricostruire xlog ---
% xlog(k,i,j): k = tempo, i = agente, j = coordinata (1=x, 2=y)

dims = size(xlog_sim);
nd = ndims(xlog_sim);

if nd == 3
    % Caso come il tuo: [2N x 1 x M]
    % es: 10 x 1 x 401  per N=5, M=401
    dim1 = dims(1);   % = 2N
    M    = dims(3);   % numero di istanti
    N    = dim1 / 2;

    xlog = zeros(M, N, 2);
    for k = 1:M
        col = xlog_sim(:,:,k);   % 2N x 1
        col = col(:).';          % 1 x 2N
        Xk  = reshape(col, 2, N).';  % N x 2
        xlog(k,:,:) = Xk;
    end

elseif nd == 2
    % Caso "classico": [M x 2N]
    % ogni riga è un istante, ogni coppia di colonne (xi_x, xi_y)
    M    = dims(1);
    dim2 = dims(2);   % = 2N
    N    = dim2 / 2;

    xlog = zeros(M, N, 2);
    for k = 1:M
        row = xlog_sim(k,:);         % 1 x 2N
        Xk  = reshape(row, 2, N).';  % N x 2
        xlog(k,:,:) = Xk;
    end

else
    error('xlog_sim has unexpected dimensions (%s).', mat2str(dims));
end

%% --- Costruzione del vettore tempo ---
t = (0:M-1)' * dt;   % [0, dt, 2dt, ..., (M-1)dt]

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

%% --- Plot della connettività λ2 ---
if opt_str == "hybrid"
    % Caso ibrido: se hai nel workspace lambda2_eps, lambda2_warn, gamma_log
    lambda2_eps  = evalin('base','lambda2_eps');
    lambda2_warn = evalin('base','lambda2_warn');
    gamma_log    = evalin('base','gamma_log');

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
    plot(t(1:m_gamma), gamma_log(1:m_gamma), 'LineWidth', 1.4);
    xlabel('time [s]'); ylabel('\gamma_{glob}'); grid on;
    title('\gamma_{glob} (gain factor) evolution');

    % Plot extra per Task Accuracy se esistono d_main e d_out
    if evalin('base','exist(''d_main'',''var'')') && evalin('base','exist(''d_out'',''var'')')
        d_main = evalin('base','d_main');
        d_out  = evalin('base','d_out');

        figure;
        set(gcf, 'Position', [500, 300, 600, 400])
        bar([d_main, d_out]);
        ylabel('Distance [m]');
        xticklabels({'Main group', 'Outlier'});
        title('Task Accuracy (respective goals)');
        grid on;
    end

else
    % Caso centralized / decentralized: solo λ2
    figure;
    plot(t, lambda2_log,'LineWidth',1.4);
    xlabel('time [s]'); ylabel('\lambda_2'); grid on;
    title('\lambda_2 (connectivity)');
end
