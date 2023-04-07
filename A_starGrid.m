function [route, Step, tot_cost] = A_starGrid (input_map, start_coords, dest_coords, drawMapEveryTime, UniformGrid)
% Run Dijkstra's algorithm on a grid.
% Basic Inputs : 
%   input_map : a logical array where the freespace cells are false or 0 and
%   the obstacles are true or 1
%   start_coords and dest_coords : Coordinates of the start and end cell
%   respectively, the first entry is the row and the second the column.
% Basic Output :
%    route : An array containing the linear indices of the cells along the
%    shortest route from start to dest or an empty array if there is no
%    route. This is a single dimensional vector
%    Step: Remember to also return the total number of nodes
%    expanded during your search. Do not count the goal node as an expanded node.

% set up color map for display
% 1 - white - clear cell
% 2 - black - obstacle
% 3 - red - visited
% 4 - blue  - on list
% 5 - green - start
% 6 - yellow - destination

cmap = [1 1 1; ... % white
        0 0 0; ... % black
        1 0 0; ... % red
        0 0 1; ... % blue
        0 1 0; ... % green
        1 1 0; ... % yellow
	0.5 0.5 0.5];  % gray

colormap(cmap);

[nrows, ncols] = size(input_map);

% map - a table that keeps track of the state of each grid cell
map = zeros(nrows,ncols);

map(~input_map) = 1;   % Mark free cells with white
map(input_map)  = 2;   % Mark obstacle cells with black

% Generate linear indices of start and dest nodes
% sub2ind = (col_index - 1) * col size + row_index = (9-1)*10+8 = 88
start_node = sub2ind(size(map), start_coords(1), start_coords(2));
goal_node  = sub2ind(size(map), dest_coords(1),  dest_coords(2));

map(start_node) = 5; % mark start node with green
map(goal_node)  = 6; % mark dest node with yellow

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% this section creates a customed cost in the map. Obstacles neighbor nodes
% take cost of 100. One of the Start node neighbor cells will be assigned a
% cost of 1000 based on the location of the start node relative to goal node

% Cost array
Cost = ones(size(input_map));
% skip this part if cost is uniform
if(~UniformGrid)
    if start_coords(1) > dest_coords(1) % start node down of the goal node
       Cost(start_coords(1)-1,start_coords(2)) = 1000;
    elseif start_coords(1) < dest_coords(1) % start node above of the goal node
       Cost(start_coords(1)+1,start_coords(2)) = 1000;
    elseif start_coords(2) > dest_coords(2) % start node right of the goal node
       Cost(start_coords(1),start_coords(2)-1) = 1000;
    elseif start_coords(2) < dest_coords(2) % start node left of the goal node
       Cost(start_coords(1),start_coords(2)+1) = 1000;
   end
   
   Cost(dest_coords(1),dest_coords(2)) = 1;
   
end
%skip this part if cost is uniform
if(~UniformGrid)
    Cost(input_map) = Inf;
    % setting cells close to obstacles with high cost of 10
    High_Cost = 100;
    for node = 1 : numel(input_map)
        % check for obstacle cells
        % if true then it is obstacle so set its neighbor at High_Cost
        if(input_map(node))
            [row, col] = ind2sub(size(map), node);
            for n = 1 : 4
                if n == 1
                    neighbor_row = row - 1; neighbor_col = col;
                elseif n == 2
                    neighbor_row = row + 1; neighbor_col = col;
                elseif n == 3
                    neighbor_row = row; neighbor_col = col - 1;
                else
                    neighbor_row = row; neighbor_col = col + 1;
                end
                
                % check neighbor is inside the map
                if (neighbor_row < 1 || neighbor_row > size(input_map,1))
                    continue
                elseif (neighbor_col < 1 || neighbor_col > size(input_map,2))
                    continue
                end
                
                neighbor = sub2ind(size(map), neighbor_row, neighbor_col);
                
                % skip if barrier OR goal_node OR start_node
                if (input_map(neighbor) == 1)
                    continue
                end
                
                if(neighbor == start_node)
                    continue
                end
                
                if(neighbor == goal_node)
                    continue
                end
                
                Cost(neighbor) = High_Cost;
            end
        end
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% this section creates a customed cost in the map. Obstacles neighbor nodes
% take cost of 100. One of the Start node neighbor cells will be assigned a
% cost of 1000 based on the location of the start node relative to goal node

% Cost array
H_func = zeros(size(input_map));
% skip this part if cost is uniform
for node = 1 : numel(input_map)
    [node_coords(1), node_coords(2)] = ind2sub(size(H_func), node);
    H_func(node) = abs(dest_coords(1)-node_coords(1)) + abs(dest_coords(2)-node_coords(2));
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initialize distance array as Inf in each cell
F_node = Inf(nrows,ncols); % This is F array in the slides
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Calculate F(start node)
F_node(start_node) = H_func(start_node) + Cost(start_node); % cost of strat node is just its H_func value
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% For each grid cell this array holds the ID of its parent
parent = zeros(nrows,ncols);

% keep track of number of nodes expanded 
Step = 0;

% Main Loop
while true
    
    % Draw current map
    map(start_node) = 5;
    map(goal_node) = 6;
    
    % make drawMapEveryTime = true if you want to see how the 
    % nodes are expanded on the grid. 
    if (drawMapEveryTime)
        pause(0.1);
        image(1.50, 1.50, map);
        grid on; % show grid
        axis image; 
        drawnow;
    end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
    % Find the node with the minimum F(node) = H(node) + Cost(node)
    % min is the vlaue of min F 
    % current_frontier is the ID of the first min value (in case there are more than one min)
    [min_F, current_frontier] = min(F_node(:)); 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % isinf  True for infinite elements. isinf(X) returns an array that contains 1's where the
    % elements of X are +Inf or -Inf and 0's where they are not.
    % For example, isinf([pi NaN Inf -Inf]) is [0 0 1 1].
    if ((current_frontier == goal_node) || isinf(min_F)) 
        break;
    end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Update map
    map(current_frontier) = 3;         % mark current node as visited (red means visited)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% we need a way so that the current frontier will not be picked from the
% array to explore in the future. Since we get new frontier that has
% minimum distance to start node so one way to mark the current fronmtier not
% to be picked again is by marking its location with Inf 
    F_node(current_frontier) = Inf; % remove this node from further consideration
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Compute row, column coordinates of current_frontier node
    [i, j] = ind2sub(size(F_node), current_frontier);
   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Visit each neighbor of the current node and update the map and parent tables appropriately.
    for n = 1 : 4 % each cell usually has 4 neighbors
        % 1 Visit each neighbor of the current_frontier node
        if n == 1
            row = i - 1; col = j;
        elseif n == 2
            row = i + 1; col = j;
        elseif n == 3
            row = i; col = j - 1;
        else
            row = i; col = j + 1;
        end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        
        % skip if outside the map
        if (row < 1 || row > size(input_map,1))
            continue
        elseif (col < 1 || col > size(input_map,2))
            continue
        end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        
        % for easy handling get ID out of coords
        neigbor = sub2ind(size(map), row, col);
        
        % skip if visited before 
        if (map(neigbor) > 1 && map(neigbor) ~= 6)%
            continue
        end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        
        % skip if barrier
        if (input_map(neigbor) == 1) % equivalent to if (map == 2) then continue 
            continue
        end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
% if not obstacle, outside map, visited before, or not previous frontier
% then update this node distance(or cost) from start and set its parent as current_frontier
        F_node(neigbor) = Cost(current_frontier) + Cost(neigbor) + H_func(neigbor); % remember min_cost is the cost of the current_frontier
                                                                                    % Cost(current_frontier) + Cost(neigbor) = distane or cost from start node
        parent(neigbor) = current_frontier;
        Step = Step + 1;    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % 2 update the map,
        if(map(neigbor) ~= 6) % goal should be always yellow
            map(neigbor) = 4; % mark neigbor with blue (to be visited) 
        end
        
        % nodes are expanded on the grid. 
        if (drawMapEveryTime)
            image(1.50, 1.50, map);
            grid on; % show grid
            axis image; 
            drawnow;
        end
        
    end
    
end

%% Construct route from start to dest by following the parent links
if (isinf(F_node(goal_node)))
    route = [];
    tot_cost = Inf;
else
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    node = goal_node;
    route = [node];
    tot_cost = 0;
    while (true)
        node = parent(node);
        route = [route node];
        if node == start_node
            break
        end
        tot_cost = tot_cost + Cost(node);
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Snippet of code used to visualize the map and the path
    for k = 2:length(route) - 1        
        map(route(k)) = 7;
        pause(0.1);
        image(1.5, 1.5, map);
        grid on;
        axis image;
    end
end

end
