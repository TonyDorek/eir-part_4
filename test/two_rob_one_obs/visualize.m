figure;
hold on;

plot(obj(1), obj(2), '-s', 'MarkerSize', 10, 'MarkerFaceColor', 'red', 'MarkerEdgeColor', 'black')
plot(x01, y01, '-o')
plot(x02, y02, '-o')

plot(x1d, y1d, '-x')
plot(x2d, y2d, '-x')

X1 = squeeze(out.X1.Data);
X2 = squeeze(out.X2.Data);

h1 = plot(NaN, NaN);
h2 = plot(NaN, NaN);

for i = 1:length(X1(1,:))
    set(h1, 'XData', X1(1,1:i), 'YData', X1(2,1:i));
    set(h2, 'XData', X2(1,1:i), 'YData', X2(2,1:i));
%    plot(X(1,:), X(2,:))
    drawnow;
    pause(0.1);
end