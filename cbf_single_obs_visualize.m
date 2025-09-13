X = squeeze(out.X.Data);
plot(X(1,:), X(2,:))
hold on;
plot(obj(1), obj(2), '-s', 'MarkerSize', 10, 'MarkerEdgeColor', 'red')