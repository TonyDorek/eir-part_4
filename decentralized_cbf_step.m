function u_vec = decentralized_cbf_step(x_vec, v_vec, u_prev_vec)
% DECENTRALIZED_CBF_STEP
%   Un singolo step di controllo decentralizzato.
%   Da usare chiamandola da Simulink.
%
%   Input:
%       x_vec      : 2N x 1 (posizioni [x1;y1;x2;y2;...])
%       v_vec      : 2N x 1 (velocità)
%       u_prev_vec : 2N x 1 (accelerazioni del passo precedente)
%
%   Output:
%       u_vec      : 2N x 1 (accelerazioni attuali)

    % --- Parametri dal workspace (initialize.m) ---
    N             = evalin('base','N');
    R_glob        = evalin('base','R_glob');
    R_loc         = evalin('base','R_loc');
    conn_margin   = evalin('base','conn_margin');
    dmin          = evalin('base','dmin');
    rsafe         = evalin('base','rsafe');
    cbf_gain_conn = evalin('base','cbf_gain_conn');
    cbf_gain_col  = evalin('base','cbf_gain_col');
    cbf_gain_obs  = evalin('base','cbf_gain_obs');
    Tpred         = evalin('base','Tpred');
    x_goal        = evalin('base','x_goal');
    obs_pos       = evalin('base','obs_pos');
    obs_rad       = evalin('base','obs_rad');
    nObs          = evalin('base','nObs');
    k_p           = evalin('base','k_p');
    k_d           = evalin('base','k_d');
    opts          = evalin('base','opts');

    % --- Rimappo in matrici N x 2 ---
    x_vec      = x_vec(:);
    v_vec      = v_vec(:);
    u_prev_vec = u_prev_vec(:);

    x      = reshape(x_vec,      2, N).';
    v      = reshape(v_vec,      2, N).';
    u_prev = reshape(u_prev_vec, 2, N).';   % N x 2

    % --- Matrice comandi da calcolare ---
    U = zeros(N,2);

    % --- QP locale per ogni agente i ---
    for i = 1:N
        % vicini entro R_glob (escludo i)
        dists = sqrt(sum((x - x(i,:)).^2,2));
        neigh = find(dists <= R_glob & ( (1:N)' ~= i ));

        % nominal acceleration
        u_nom_i = u_nom_fun(k_p, k_d, x(i,:), v(i,:), x_goal);  % 1x2

        % vincoli locali: A_i u_i <= b_i  (u_i è 2x1)
        A_i = [];
        b_i = [];

        % --- vincoli agent-agent ---
        for j = neigh'
            xij = x(i,:) - x(j,:);
            vij = v(i,:) - v(j,:);
            u_j = u_prev(j,:);        % accel del vicino al passo precedente
            dij = norm2(xij);

            % collision avoidance (predicted)
            xT   = xij + Tpred * vij;
            h_col = (xT * xT.') - dmin^2;

            arow = -2*Tpred * xT;     % 1x2
            brow =  2*(xT * vij.') ...
                    - 2*Tpred*(xT * u_j.') ...
                    + cbf_gain_col * h_col;

            A_i = [A_i; arow];
            b_i = [b_i; brow];

            % connectivity (solo vicino al bordo R_loc)
            if dij < (R_loc + conn_margin)
                xT_c  = xij + Tpred * vij;
                h_conn = R_loc^2 - (xT_c * xT_c.');

                arow_c =  2*Tpred * xT_c;
                brow_c = -2*(xT_c * vij.') ...
                         + 2*Tpred*(xT_c * u_j.') ...
                         + cbf_gain_conn * h_conn;

                A_i = [A_i; arow_c];
                b_i = [b_i; brow_c];
            end
        end

        % --- obstacle avoidance ---
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

        % --- solve local 2D QP: min ||u_i - u_nom_i||^2 ---
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
