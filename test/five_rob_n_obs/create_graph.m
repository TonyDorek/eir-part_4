function [A, G, L, p] = create_graph(state, range, text)
% The function plots a graph 2D robot positions
% The inputs are the state of the system and the connectivity range
% The outputs are the matrices associated to the graph and the graph plot

N = numel(state)/2;
pos = reshape(state,2,N)';

% Define a polinomial Adjacency matrix
A = zeros(N);
for i=1:N-1
    xi = pos(i,:);
    for j=i+1:N
        xj = pos(j,:);
        rij = xi - xj;
        dij = sqrt( max( rij(1)^2 + rij(2)^2, 0 ) );   % non-negative
        if dij <= range   % polynomial bump (smooth, bounded)
            t = (dij / range)^2;
            aij = (1 - t)^3;   % in [0,1]
        else
            aij = 0;
        end
        A(i,j) = aij;
        A(j,i) = aij;
    end
end

% Compute the Degree matrix
degreeVector = sum(A, 2); % Sum of rows gives the degree of each node
D = diag(degreeVector);

% Compute Laplacian matrix and its eigenvector/eigenvalues
L = D - A;
% [autovec, autoval] = eig(L);

% Create and plot the graph with customizations
G = graph(A);
p = plot(G, 'XData', state(1,:), 'YData', state(2,:), 'EdgeLabel', G.Edges.Weight);
p.LineWidth = 2; % Set edge line width
p.NodeColor = 'r'; % Set node color
p.MarkerSize = 8; % Set node marker size

% Add title and axis labels
title(text);
xlabel('x');
ylabel('y');

% Add legend
% legend('graph','Location','northwest');

end

