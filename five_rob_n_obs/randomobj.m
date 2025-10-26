function Generated_Objects = randomobj(N,spawnrange)
    % N is how many objects we need to generate positions for
    % x_lb, y_lb are lower bounds for x and y (default is -100)
    % x_ub, y_ub are upper bounds for x and y (default is 100)
    arguments
        N (1,1) double {mustBePositive, mustBeInteger}
        spawnrange (1,1) double = 50
    end

    x_lb = -spawnrange/2;
    x_ub = spawnrange/2;
    y_lb = -spawnrange/2;
    y_ub = spawnrange/2;



    for i = 1:N
        x = x_lb + (x_ub - x_lb) * rand();
        y = y_lb + (y_ub - y_lb) * rand();
        Generated_Objects(:, i) = [x; y];
    end
end 



