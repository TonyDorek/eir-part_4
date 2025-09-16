figure;
hold on;

plot(obj(1), obj(2), '-s', 'MarkerSize', 10, 'MarkerFaceColor', 'red', 'MarkerEdgeColor', 'black')
plot(x0, y0, '-o')
plot(xd, yd, '-x')

X = squeeze(out.X.Data);
h = plot(NaN, NaN);
for i = 1:length(X(1,:))
    set(h, 'XData', X(1,1:i), 'YData', X(2,1:i));
%    plot(X(1,:), X(2,:))
    drawnow;
    pause(0.1);
end