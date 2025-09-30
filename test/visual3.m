figure; hold on; grid on; axis equal;
title('Animazione traiettorie');
xlabel('X'); ylabel('Y');

plot(obj1(1), obj1(2), '-s', 'MarkerSize', 10, 'MarkerFaceColor', 'red', 'MarkerEdgeColor', 'black');
plot(obj2(1), obj2(2), '-s', 'MarkerSize', 10, 'MarkerFaceColor', 'blue', 'MarkerEdgeColor', 'black');


plot(x01, y01, '-o', 'MarkerFaceColor', 'k');
plot(x02, y02, '-o', 'MarkerFaceColor', 'k');
plot(x03, y03, '-o', 'MarkerFaceColor', 'k');

plot(x1d, y1d, '-x', 'MarkerSize', 8, 'LineWidth', 1.5);
plot(x2d, y2d, '-x', 'MarkerSize', 8, 'LineWidth', 1.5);
plot(x3d, y3d, '-x', 'MarkerSize', 8, 'LineWidth', 1.5);


h1 = plot(NaN, NaN, 'r', 'LineWidth', 2);
h2 = plot(NaN, NaN, 'b', 'LineWidth', 2);
h3 = plot(NaN, NaN, 'g', 'LineWidth', 2);

legend('Obj1','Obj2','Start1','Start2','Start3','Goal1','Goal2','Goal3','Traj1','Traj2','Traj3');


for i = 1:length(X1(1,:))
    set(h1, 'XData', X1(1,1:i), 'YData', X1(2,1:i));
    set(h2, 'XData', X2(1,1:i), 'YData', X2(2,1:i));
    set(h3, 'XData', X3(1,1:i), 'YData', X3(2,1:i));
    drawnow;
    pause(0.05);
end