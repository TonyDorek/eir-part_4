function u_vec = hybrid_cbf_step(x_vec, v_vec, u_prev_vec)
% HYBRID_CBF_STEP
%   Un singolo step del controllo ibrido:
%   - CBF decentralizzate (collisioni / connettività / ostacoli)
%   - feedback globale via λ2 che modula la connettività
%   - un agente "speciale" con goal alternativo quando la conn. è ok
%
%   Input:
%       x_vec      : 2N x 1, posizioni
%       v_vec      : 2N x 1, velocità
%       u_prev_vec : 2N x 1, accelerazioni (passo precedente)
%
%   Output:
%       u_vec      : 2N x 1, accelerazioni attuali

    % ----- Parametri dal workspace -----
    N               = evalin('base','N');
    R_glob          = evalin('base','R_glob');
    R_loc           = evalin('base','R_loc');
    conn_margin     = evalin('base','conn_margin');
    dmin            = evalin('base','dmin');
    rsafe           = evalin('base','rsafe');
    cbf_gain_conn   = evalin('base','cbf_gain_conn');
    cbf_gain_col    = evalin('base','cbf_gain_col');
    cbf_gain_obs    = evalin('base','cbf_gain_obs');
    Tpred           = evalin('base','Tpred');
    k_p             = evalin('base','k_p');
    k_d             = evalin('base','k_d');
    x_goal          = evalin('base','x_goal');
    x_goal_alt      = evalin('base','x_goal_alt');
    special_idx     = evalin('base','special_idx');
    G               = evalin('base','G'); %#ok<NASGU> % se ti serve offline
    obs_pos         = evalin('base','obs_pos');
    obs_rad         = evalin('base','obs_rad');
    nObs            = evalin('base','nObs');
    opts            = evalin('base','opts');

    lambda2_eps     = evalin('base','lambda2_eps');
    lambda2_warn    = evalin('base','lambda2_warn');
    k_lambda_glob   = evalin('base','k_lambda_glob');
    gamma_max       = evalin('base','gamma_max');
    lambda_fb_act   = evalin('base','lambda_feedback_active');  % flag precedente

    % ----- Rimappo vettori in matrici N x 2 -----
    x_vec      = x_vec(:);
    v_vec      = v_vec(:);
    u_prev_vec = u_prev_vec(:);

    x      = reshape(x_vec,      2, N).';
    v      = reshape(v_vec,      2, N).';
    u_prev = reshape(u_prev_vec, 2, N).';

    % ==========================================================
    % 1) Calcolo globale di λ2 (come nel codice ibrido)
    % ==========================================================
    Aconn = zeros(N);
    for i = 1:N-1
        for j = i+1:N
            dist = norm2(x(i,:) - x(j,:));
            if dist <= R_glob
                Aconn(i,j) = incmat_com(dist, R_glob);
                Aconn(j,i) = Aconn(i,j);
            end
        end
    end
    L  = diag(sum(Aconn,2)) - Aconn;
    ev = sort(eig(L));
    if numel(ev) >= 2
        lambda2 = ev(2);
    else
        lambda2 = 0;
    end
    lambda2_hat = lambda2;  % stima distribuita ≈ valore reale

    % ---- Aggiorno flag lambda_feedback_active ----
    if (~lambda_fb_act) && (lambda2_hat > lambda2_eps)
        lambda_fb_act = true;
    end

    % ---- Guadagno globale gamma_glob ----
    if lambda_fb_act
        if lambda2_hat < lambda2_warn
            gamma_glob = 1 + k_lambda_glob * (lambda2_warn - lambda2_hat);
            gamma_glob = min(gamma_glob, gamma_max);
        else
            gamma_glob = 1.0;
        end
    else
        gamma_glob = 1.0;
    end

    % gain effettivo sulla connettività
    cbf_gain_conn_eff = cbf_gain_conn * gamma_glob;

    % Salvo il nuovo valore del flag nel workspace (per lo step successivo)
    assignin('base','lambda_feedback_active',lambda_fb_act);

    % ==========================================================
    % 2) QP locale per ogni agente (come decentr., ma con gamma e special goal)
    % ==========================================================
    U = zeros(N,2);

    for i = 1:N
        % --- goal dipendente da agente/flag ---
        if (i == special_idx) && lambda_fb_act
            goal_i = x_goal_alt;
        else
            goal_i = x_goal;
        end

        % nominale PD
        u_nom_i = -k_p * (x(i,:) - goal_i) - k_d * v(i,:);

        % vicini entro raggio
        dists = sqrt(sum((x - x(i,:)).^2,2));
        neigh = find(dists <= R_glob & ( (1:N)' ~= i ));

        % vincoli locali: A_i u_i <= b_i
        A_i = [];
        b_i = [];

        for j = neigh'
            xij = x(i,:) - x(j,:);
            vij = v(i,:) - v(j,:);
            u_j = u_prev(j,:);
            dij = norm2(xij);

            % --- collision avoidance (predicted) ---
            xT   = xij + Tpred * vij;
            h_col = (xT * xT.') - dmin^2;

            arow = -2*Tpred * xT;
            brow =  2*(xT * vij.') ...
                    - 2*Tpred*(xT * u_j.') ...
                    + cbf_gain_col * h_col;

            A_i = [A_i; arow];
            b_i = [b_i; brow];

            % --- connettività locale (predicted) ---
            if dij < (R_loc + conn_margin)
                xT_c  = xij + Tpred * vij;
                h_conn = R_loc^2 - (xT_c * xT_c.');

                arow_c =  2*Tpred * xT_c;
                brow_c = -2*(xT_c * vij.') ...
                         + 2*Tpred*(xT_c * u_j.') ...
                         + cbf_gain_conn_eff * h_conn;

                A_i = [A_i; arow_c];
                b_i = [b_i; brow_c];
            end
        end

        % --- ostacoli ---
        for o = 1:nObs
            xio  = x(i,:) - obs_pos(o,:);
            vio  = v(i,:);
            xT_o = xio + Tpred * vio;
            h_obs = (xT_o * xT_o.') - (obs_rad(o) + rsafe)^2;

            arow_o = -2*Tpred * xT_o;
            brow_o =  2*(xT_o * vio.') + cbf_gain_obs * h_obs;

            A_i = [A_i; arow_o];
            b_i = [b_i; brow_o];
        end

        % --- QP: min ||u_i - u_nom_i||^2 ---
        H = 2*eye(2);
        f = -2*u_nom_i';

        if isempty(A_i)
            u_i = u_nom_i';
        else
            [u_i,~,flag] = quadprog(H,f,A_i,b_i,[],[],[],[],[],opts);
            if flag <= 0 || isempty(u_i)
                u_i = u_nom_i';
            end
        end

        U(i,:) = u_i';
    end

    % --- Output vettoriale 2N x 1 ---
    u_vec = reshape(U.', [], 1);
end
