% --- Find Connected Component ---
function component_robots = find_connected_component(Aconn, N, robot_id)
    % Breadth-First Search to find connected component containing robot_id
    
    visited = false(N, 1);
    queue = robot_id;
    visited(robot_id) = true;
    component_robots = robot_id;
    
    while ~isempty(queue)
        current = queue(1);
        queue(1) = [];
        
        for neighbor = 1:N
            if Aconn(current, neighbor) > 0 && ~visited(neighbor)
                visited(neighbor) = true;
                queue = [queue, neighbor];
                component_robots = [component_robots, neighbor];
            end
        end
    end
end