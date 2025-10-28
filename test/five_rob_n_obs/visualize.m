%% === OUTPUT VISUALIZATION ===
fig = figure; % Store figure handle
set(fig, 'Position', [250, 250, 1200, 500]); % [x, y, width, height]
subplot(1, 2, 1);
hold on; grid on;
title('Safe multi-agent system: collision avoidance + connectivity maintenance');
xlabel('x'); ylabel('y');
colors = ['r','b','g','m','c'];

% --- Plotting obstacles ---
obstacle_colors = ['r', 'b', 'k', 'y', 'm'];  % Colors for different obstacles
for i = 1:n_obj
    plot(obstacles(1,i), obstacles(2,i), '-s', 'MarkerSize', 10, ...
         'MarkerFaceColor', obstacle_colors(mod(i-1,5)+1), 'MarkerEdgeColor', 'black');
    text(obstacles(1,i), obstacles(2,i), ['obj' num2str(i)], ...
         'HorizontalAlignment', 'left', 'VerticalAlignment', 'top');
end

% --- Plotting initial positions ---
for i = 1:n_robots
    plot(X0(1,i), X0(2,i), '-o', 'MarkerFaceColor', colors(i));
    text(X0(1,i), X0(2,i), strcat('x0', num2str(i)), ...
         'HorizontalAlignment', 'left', 'VerticalAlignment', 'top');
end

% --- Plotting desired positions ---
for i = 1:n_robots
    plot(Xd(1,i), Xd(2,i), '-x', 'MarkerSize', 8, 'LineWidth', 1.5, 'MarkerEdgeColor', colors(i));
    text(Xd(1,i), Xd(2,i), strcat('xd', num2str(i)), ...
         'HorizontalAlignment', 'left', 'VerticalAlignment', 'top');
end

% --- Recovering and handling trajectories (from Simulink simulations) ---
for i = 1:n_robots
    eval(sprintf('X%d = squeeze(out.X%d.Data);', i, i))
end

h = gobjects(1, n_robots);
for i = 1:n_robots
    h(i) = plot(NaN, NaN, colors(i), 'LineWidth', 2);
end

% --- System dynamics ---
time_index = length(X1(1,:));

for i = 1:time_index
    for j = 1:n_robots
        set(h(j), 'XData', eval(sprintf('X%d(1,1:i)',j)), 'YData', eval(sprintf('X%d(2,1:i)',j)));
    end
    drawnow;
    pause(0.05);
end

subplot(1, 2, 2);
% grid on;

% --- Graph dynamics ---
v = zeros(2, n_robots);

for i = 1:time_index
    if ~isvalid(fig) || ~ishandle(fig) % Check if figure is still open
        disp('Figure closed. Animation stopped.');
        break;
    end
    for j = 1:n_robots
        v(:,j) = eval(sprintf('X%d(:,i)', j));
    end
    [~,p] = create_graph(v, range, 'Evolution of robot positions (nodes) and connections (edges)');
    drawnow;
    pause(0.05);
end