clear;
clc;

%% === PARAMETRI INIZIALI ===
n_robots = 5;           
thr = 2;               

kx = 0.6 * ones(1, n_robots);
ky = 0.6 * ones(1, n_robots);

for i = 1:n_robots
    K{i} = [kx(i), 0;
             0, ky(i)];
end

%% === POSIZIONI INIZIALI ===
X0 = [  0, 102,  50, 160, 200;   
        0,  56, -20,  40, 100];  

%% === POSIZIONI DESIDERATE ===
Xd = [102,   2,  40, 180, 220;    
       56,   2,  60,  80, 120];   

%% === DERIVATE DESIDERATE ===
Xd_dot = zeros(2, n_robots);  

%% === OSTACOLI ===
obj1 = [60, 45];  
obj2 = [30, 25];  

%% === ANIMAZIONE ===
figure; hold on; grid on; axis equal;
title('Animazione traiettorie (5 Robot, 2 Ostacoli)');
xlabel('X'); ylabel('Y');

% --- Ostacoli ---
plot(obj1(1), obj1(2), '-s', 'MarkerSize', 10, 'MarkerFaceColor', 'red', 'MarkerEdgeColor', 'black');
plot(obj2(1), obj2(2), '-s', 'MarkerSize', 10, 'MarkerFaceColor', 'blue', 'MarkerEdgeColor', 'black');

% --- Posizioni iniziali ---
for i = 1:n_robots
    plot(X0(1,i), X0(2,i), '-o', 'MarkerFaceColor', 'k');
end

% --- Posizioni finali ---
for i = 1:n_robots
    plot(Xd(1,i), Xd(2,i), '-x', 'MarkerSize', 8, 'LineWidth', 1.5);
end

% --- Traiettorie (da Simulink o simulazione) ---
X1 = squeeze(out.X1.Data);
X2 = squeeze(out.X2.Data);
X3 = squeeze(out.X3.Data);
X4 = squeeze(out.X4.Data);
X5 = squeeze(out.X5.Data);

% --- Handle delle traiettorie ---
colors = ['r','b','g','m','c'];
h = gobjects(1, n_robots);
for i = 1:n_robots
    h(i) = plot(NaN, NaN, colors(i), 'LineWidth', 2);
end

% --- Legenda ---
legend('Obj1','Obj2', ...
       'Start1','Start2','Start3','Start4','Start5', ...
       'Goal1','Goal2','Goal3','Goal4','Goal5', ...
       'Traj1','Traj2','Traj3','Traj4','Traj5');

%% === ANIMAZIONE DINAMICA ===
% (qui si suppone che tutti i vettori Xi abbiano stessa lunghezza)
for i = 1:length(X1(1,:))
    set(h(1), 'XData', X1(1,1:i), 'YData', X1(2,1:i));
    set(h(2), 'XData', X2(1,1:i), 'YData', X2(2,1:i));
    set(h(3), 'XData', X3(1,1:i), 'YData', X3(2,1:i));
    set(h(4), 'XData', X4(1,1:i), 'YData', X4(2,1:i));
    set(h(5), 'XData', X5(1,1:i), 'YData', X5(2,1:i));
    drawnow;
    pause(0.2);
end
