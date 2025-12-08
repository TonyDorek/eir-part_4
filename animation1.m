%% ANIMAZIONE DINAMICA ROBOT (senza Image Processing toolbox)

clc;
figure; hold on; grid on; axis equal;

dt      = evalin('base','dt');
obs_pos = evalin('base','obs_pos');
obs_rad = evalin('base','obs_rad');

%% ricostruzione xlog (come in visualize)
dims = size(xlog_sim);
nd = ndims(xlog_sim);

if nd == 3
    dim1 = dims(1);
    M    = dims(3);
    N    = dim1 / 2;

    xlog = zeros(M, N, 2);
    for k = 1:M
        col = xlog_sim(:,:,k);  
        col = col(:).';         
        Xk  = reshape(col, 2, N).';  
        xlog(k,:,:) = Xk;
    end
else
    M    = dims(1);
    dim2 = dims(2);
    N    = dim2 / 2;

    xlog = zeros(M, N, 2);
    for k = 1:M
        row = xlog_sim(k,:);
        Xk  = reshape(row, 2, N).';  
        xlog(k,:,:) = Xk;
    end
end

%% ANIMAZIONE
for k = 1:M
    if mod(k, 5)==0
        clf; hold on; grid on; axis equal;
        XK = squeeze(xlog(k,:,:)); % Nx2
    
        axis([-8 8 -8 8]);
        xlabel('x [m]'); ylabel('y [m]');
        title(sprintf('Time = %.2f s', (k-1)*dt));
    
        % --- ostacoli ---
        for o = 1:length(obs_rad)
            draw_circle(obs_pos(o,:), obs_rad(o), 'r');    % bordo ostacolo
        end
    
            % --- connessioni tra robot (se dist <= R_glob) ---
        R_glob = evalin('base','R_glob');  % se non l'hai già preso all'inizio
    
        for i = 1:N-1
            for j = i+1:N
                dist = norm(XK(i,:) - XK(j,:));
                if dist <= R_glob
                    plot([XK(i,1), XK(j,1)], ...
                         [XK(i,2), XK(j,2)], 'k-', 'LineWidth', 0.5);
                end
            end
        end
  


        % --- robot ---
        scatter(XK(:,1), XK(:,2), 140, 'b', 'filled');
    
        drawnow;
        pause(0.03);   % velocità dell'animazione
 end
end 
%% Funzione per disegnare cerchi senza toolbox
function draw_circle(center, radius, color)
    t = linspace(0,2*pi,100);
    x = center(1) + radius*cos(t);
    y = center(2) + radius*sin(t);
    plot(x,y, color, 'LineWidth', 1.8);
end
