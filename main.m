clear all
close all

% % Ex: 1
% map = false(10);
% map (1:5, 5:8) = true;
% start_coords = [1, 1];
% goal_coords  = [10, 10];

% % Ex: 2
% map = false(10);
% map (1:5, 6) = true;
% start_coords = [6, 2];
% goal_coords  = [8, 9];

% % Ex: 3
% map = false(10);
% map (1:5, 5:8) = true;
% start_coords = [1, 1];
% goal_coords  = [1, 10];

% Ex: 3
map = false(10);
%map (1:5, 5:8) = true;
start_coords = [1, 1];
goal_coords  = [1, 10];

% % Ex: 4
% % create a 10x5 map filled with zeros (free cells)
% map = false(10,5); 
% % mark obstacles
% map (2:10, 1) = true;
% map (1:2, 4:5) = true;
% map (10, 2:3) = true;
% % assign start and goal node
% start_coords = [7, 3];
% goal_coords  = [2, 3];
% %goal_coords  = [10, 5];

% % Ex: 5
% % create a 10x5 map filled with zeros (free cells)
% map = false(10,5); 
% % mark obstacles
% map (:, 1) = true;
% map (1, 5) = true;
% map (10, 2:3) = true;
% map (2:3, 3) = true;
% % assign start and goal node
% start_coords = [7, 4];
% goal_coords  = [3, 2];

% % Ex: 6
% % create a 10x5 map filled with zeros (free cells)
% map = false(10,5); 
% % mark obstacles
% map (:, 1) = true;
% map (1, 5) = true;
% map (10, 2:3) = true;
% map (2:3, 3) = true;
% % assign start and goal node
% start_coords = [7, 2];
% goal_coords  = [7, 4];

% % Ex: 7 - No route check
% map = false(10,5); 
% map (:, 1) = true;
% map (1, 5) = true;
% map (10, 2:3) = true;
% map (4, 2:5) = true;
% map (2:3, 3) = true;
% start_coords = [7, 4];
% goal_coords  = [3, 2];

% Boolian variable to control if the map is being visualized on every iteration
drawMapEveryTime = true;

% Boolian variable to select whether unifrom or non-uniform grid
UniformGrid = true;

% string variable to choose algorithm
algorithm = 'A' % A->A_star, D->Dijkstra 

if algorithm == 'D'
    [route, Step, tot_cost] = DijkstraGrid(map,start_coords,goal_coords,drawMapEveryTime,UniformGrid)
elseif algorithm == 'A'
    [route, Step, tot_cost] = A_starGrid(map,start_coords,goal_coords,drawMapEveryTime,UniformGrid)
end
