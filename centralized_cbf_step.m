function u_vec = centralized_cbf_step(x_vec, v_vec)
% CENTRALIZED_CBF_STEP
%   Un singolo step del controllo CBF centralizzato.
%   Da usare chiamandola da Simulink.

    % ---------------------------------------------------------------------
    % 1) Parametri dal workspace (set in initialize.m)
    % ---------------------------------------------------------------------
    N             = evalin('base', 'N');
    R_glob        = evalin('base', 'R_glob');
    R_loc         = evalin('base', 'R_loc');
    conn_margin   = evalin('base', 'conn_margin');
    dmin          = evalin('base', 'dmin');
    rsafe         = evalin('base', 'rsafe');
    cbf_gain_conn = evalin('base', 'cbf_gain_conn');
    cbf_gain_col  = evalin('base', 'cbf_gain_col');
    cbf_gain_obs  = evalin('base', 'cbf_gain_obs');
    Tpred         = evalin('base', 'Tpred');
    x_goal        = evalin('base', 'x_goal');
    obs_pos       = evalin('base', 'obs_pos');
    obs_rad       = evalin('base', 'obs_rad');
    nObs          = evalin('base', 'nObs');
    k_p           = evalin('base', 'k_p');
    k_d           = evalin('base', 'k_d');
    opts          = evalin('base', 'opts');

    % ---------------------------------------------------------------------
    % 2) Ricostruisco x e v in matrici N x 2
    % ---------------------------------------------------------------------
    x_vec = x_vec(:);   % forzo colonna
    v_vec = v_vec(:);

    x = reshape(x_vec, 2, N).';  % N x 2
    v = reshape(v_vec, 2, N).';  % N x 2

    % ---------------------------------------------------------------------
    % 3) Accelerazioni nominali u_nom (PD verso x_goal)
    % ---------------------------------------------------------------------
    u_nom = zeros(2*N, 1);
    for i = 1:N
        idx = vecIdx(i);  % es: [1 2], [3 4], ...
        u_nom(idx) = u_nom_fun(k_p, k_d, x(i,:), v(i,:), x_goal).';
    end

    % ---------------------------------------------------------------------
    % 4) Costruzione vincoli CBF: Arows * u <= brows
    % ---------------------------------------------------------------------
    Arows = [];
    brows = [];

    % ---- (a) Collisioni e connettività tra agenti -----------------------
    for i = 1:N-1
        for j = i+1:N
            xij = x(i,:) - x(j,:);
            vij = v(i,:) - v(j,:);
            dij = norm2(xij);

            % --- Collision avoidance (predicted) ---
            xT = xij + Tpred * vij;
            h_col = (xT * xT.') - dmin^2;

            idx_i = vecIdx(i);
            idx_j = vecIdx(j);

            arow_col = zeros(1, 2*N);
            vec_c    = -2 * Tpred * xT;

            arow_col(idx_i) =  vec_c;
            arow_col(idx_j) = -vec_c;

            brow_col = 2*(xT * vij.') + cbf_gain_col * h_col;

            Arows = [Arows; arow_col];
            brows = [brows; brow_col];

            % --- Connectivity (near R_loc) ---
            if dij < (R_loc + conn_margin)
                xT_conn = xij + Tpred * vij;
                h_conn  = R_loc^2 - (xT_conn * xT_conn.');

                arow_conn = zeros(1, 2*N);
                vecc      = 2 * Tpred * xT_conn;

                arow_conn(idx_i) =  vecc;
                arow_conn(idx_j) = -vecc;

                brow_conn = -2*(xT_conn * vij.') + cbf_gain_conn * h_conn;

                Arows = [Arows; arow_conn];
                brows = [brows; brow_conn];
            end
        end
    end

    % ---- (b) Collisioni agente–ostacolo --------------------------------
    for i = 1:N
        idx_i = vecIdx(i);
        for o = 1:nObs
            xio  = x(i,:) - obs_pos(o,:);
            vio  = v(i,:);
            xT_o = xio + Tpred * vio;

            h_obs = (xT_o * xT_o.') - (obs_rad(o) + rsafe)^2;

            arow_obs = zeros(1, 2*N);
            vec_o    = -2 * Tpred * xT_o;

            arow_obs(idx_i) = vec_o;
            brow_obs = 2*(xT_o * vio.') + cbf_gain_obs * h_obs;

            Arows = [Arows; arow_obs];
            brows = [brows; brow_obs];
        end
    end

    % ---------------------------------------------------------------------
    % 5) QP: min ||u - u_nom||^2  s.t. Arows*u <= brows
    % ---------------------------------------------------------------------
    H = 2*eye(2*N);
    f = -2*u_nom;

    if isempty(Arows)
        u_sol = u_nom;
    else
        H = H + 1e-6*eye(2*N);
        [u_sol,~,exitflag] = quadprog(H, f, Arows, brows, [], [], [], [], [], opts);
        if exitflag <= 0 || isempty(u_sol)
            warning('QP infeasible in centralized_cbf_step, uso u_nom.');
            u_sol = u_nom;
        end
    end

    % ---------------------------------------------------------------------
    % 6) Output
    % ---------------------------------------------------------------------
    u_vec = u_sol;
end
