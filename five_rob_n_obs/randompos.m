function Generated_Positions = randompos(N,thr,spawnrange)
    % N is how many robots we need to generate positions for
    % x_lb, y_lb are lower bounds for x and y (default is -100)
    % x_ub, y_ub are upper bounds for x and y (default is 100)
    arguments
        N (1,1) double {mustBePositive, mustBeInteger}
        thr (1,1) double = 5
        spawnrange (1,1) double = 50

    end

    x_lb = -spawnrange/2;
    x_ub = spawnrange/2;
    y_lb = -spawnrange/2;
    y_ub = spawnrange/2;



    


    safetymargin=1; 
    min_distance = thr + safetymargin; %mins spawn distance between starts
    Generated_Positions = ones(2,N) * (max(abs(x_ub), abs(y_ub)) + min_distance);
    max_attempts = 10000; % Maximum attempts per position

       for i = 1:N
        valid_position = false;
        attempts = 0;
        
        while ~valid_position && attempts < max_attempts
            % Generate random position
            x = x_lb + (x_ub - x_lb) * rand();
            y = y_lb + (y_ub - y_lb) * rand();
            
            % Check distance from all existing positions
            if i == 1
                % First position is always valid
                valid_position = true;
            else
                % Calculate distances to all existing positions
                distances = sqrt((Generated_Positions(1, 1:i-1) - x).^2 + (Generated_Positions(2, 1:i-1) - y).^2);
                
                % Check if all distances are at least min_distance
                if all(distances >= min_distance)
                    valid_position = true;
                end
            end
            
            attempts = attempts + 1;
        end
        
        if ~valid_position
            error('Could not find valid position after %d attempts. Consider increasing bounds or decreasing N/thr.', max_attempts);
        end
        
        Generated_Positions(:, i) = [x; y];
    end



end