%% === ANIMATIONS ===
figure; 

subplot(1, 2, 1);

hold on; grid on;% axis equal;
 
set(gcf, 'Position', [250, 250, 1200, 500]); % [x, y, width, height]

title('Safe multi-agent system: collision avoidance + connectivity maintenance');
xlabel('x'); ylabel('y');

% --- Obstacles ---
plot(obj1(1), obj1(2), '-s', 'MarkerSize', 10, 'MarkerFaceColor', 'red', 'MarkerEdgeColor', 'black');
plot(obj2(1), obj2(2), '-s', 'MarkerSize', 10, 'MarkerFaceColor', 'blue', 'MarkerEdgeColor', 'black');

text(obj1(1), obj1(2), 'obj1', 'HorizontalAlignment', 'left', 'VerticalAlignment', 'top');
text(obj2(1), obj2(2), 'obj2', 'HorizontalAlignment', 'left', 'VerticalAlignment', 'top');

colors = ['r','b','g','m','c'];

% --- Initial positions ---
for i = 1:n_robots
    plot(X0(1,i), X0(2,i), '-o', 'MarkerFaceColor', colors(i));
    text(X0(1,i), X0(2,i), strcat('x0', num2str(i)), 'HorizontalAlignment', 'left', 'VerticalAlignment', 'top');
end

% --- Final positions ---
for i = 1:n_robots
    plot(Xd(1,i), Xd(2,i), '-x', 'MarkerSize', 8, 'LineWidth', 1.5, 'MarkerEdgeColor', colors(i));
    text(Xd(1,i), Xd(2,i), strcat('xd', num2str(i)), 'HorizontalAlignment', 'left', 'VerticalAlignment', 'top');
end

% --- Trajectories (from Simulink simulations) ---
X1 = squeeze(out.X1.Data);
X2 = squeeze(out.X2.Data);
X3 = squeeze(out.X3.Data);
X4 = squeeze(out.X4.Data);
X5 = squeeze(out.X5.Data);

% --- Handling trajectories ---
h = gobjects(1, n_robots);
for i = 1:n_robots
    h(i) = plot(NaN, NaN, colors(i), 'LineWidth', 2);
end

% --- Legenda ---
% legend('Obj1','Obj2', ...
%        'Start1','Start2','Start3','Start4','Start5', ...
%        'Goal1','Goal2','Goal3','Goal4','Goal5', ...
%        'Traj1','Traj2','Traj3','Traj4','Traj5');

% --- System dynamics ---
for i = 1:length(X1(1,:))
    set(h(1), 'XData', X1(1,1:i), 'YData', X1(2,1:i));
    set(h(2), 'XData', X2(1,1:i), 'YData', X2(2,1:i));
    set(h(3), 'XData', X3(1,1:i), 'YData', X3(2,1:i));
    set(h(4), 'XData', X4(1,1:i), 'YData', X4(2,1:i));
    set(h(5), 'XData', X5(1,1:i), 'YData', X5(2,1:i));
    drawnow;
    pause(0.2);
end

subplot(1, 2, 2); 

grid on;

% --- Graph dynamics ---

for i = 1:length(X1(1,:))
    [~,p] = create_graph([X1(:,i), X2(:,i), X3(:,i), X4(:,i), X5(:,i)],range, ...
        'Evolution of robot positions (nodes) and connections (edges)');
    drawnow;
    pause(0.2);
end
