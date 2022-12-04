%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Simple simulation of N nodes using the weight-swapping controller. Define your
% initial positions in `x`. Edges are assigned using a Delta-disk proximity
% model.
%
% Author: Andrew Glick
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

rowmag = @(A) sqrt(sum(A.^2,2));  % Magnitude of each row
bound = @(A, lower, upper) min(max(A,lower),upper);
edge_len = @(x, i, j) norm(x(j,:)-x(i,:));
fi = @(varargin)varargin{length(varargin)-varargin{1}};

window = [0 0 6 6]; % (x, y, w, h)
border = 0.5;
iterations = 2000;

close_enough = 0.1;

min_dist = 0.6;
max_dist = 1.5;
dx_max = 0.04;
dx_gain = 0.2;

x = [1 2; 2 2; 3 3; 3 1; 4 2; 4 3];  % Position
dx = zeros(size(x)); % Velocity
N = length(x);  % Num nodes

isLeader= false(N, 1);
isLeader([1, end]) = true;
isFixed = false(N, 1);  % Bool mask of which nodes are fixed
isFixed(1) = true;

edges = proximity_graph(x, max_dist);
weights = ones(height(edges), 1);

isExpanding = true;
super_leader = 1;  % Node responsible for controlling expansion

G = graph(edges(:,1), edges(:,2), weights, N);

% Setup plotting
f = figure(1);
clf(f);
p = plot(G, XData=x(:,1), YData=x(:,2), ...
    EdgeColor="black" ...
);
hold on
% Velocity vectors for convenience
% q = quiver(x(:,1), x(:,2), dx(:,1), dx(:,2), 0.5);
hold off
debug_box = annotation( ...
    textbox=[0.2, 0.6, 0.3, 0.3],...
    String="", ...
    FitBoxToText="on" ...
);
rectangle(Position=window);
xlim([-border, window(3)+border]+window(1));
ylim([-border, window(4)+border]+window(2));
axis equal

% Run simulation
for t = 1:iterations
    % Reset dx
    dx(:,:) = 0;

    % Update graph
    new_edges = proximity_graph(x, max_dist/2, G);
    if ~isempty(new_edges)
        G = addedge(G, new_edges(:,1), new_edges(:,2), length(new_edges));
        % Replot
        p = plot(G, XData=x(:,1), YData=x(:,2), ...
            EdgeColor="black" ...
        );
%         rectangle(Position=window);
        xlim([-border, window(3)+border]+window(1));
        ylim([-border, window(4)+border]+window(2));
    end

    % Consensus
    for i = 1:N
        % Skip fixed nodes
        if isFixed(i)
            continue
        end

        % Neighbors
        for j = neighbors(G, i)'  % Transpose because Matlab is dumb
            w = G.Edges.Weight(findedge(G, i, j));
            
            % Set target distance
            if isExpanding && any(isLeader([i, j]))
                target_dist = max_dist;

                % If J is a leader, then put less weight on this side of
                % the edge to prevent collisions.
                if isLeader(j)
                    w = w * 0.1;
                end
            else
                target_dist = min_dist;
            end

            dx(i,:) = dx(i,:) + dx_gain*abs(w)*(edge_len(x, i, j)^2 - target_dist^2)*(x(j,:)-x(i,:));
        end
    end


    % Mode swapping. The state of the system is fully controlled by E(1,2)
    
    % Get all edge lengths of the super_leader
    super_leader_edge_lens = arrayfun(@(j) edge_len(x, super_leader, j), neighbors(G, super_leader));

    % Done expanding
    if isExpanding && (max(super_leader_edge_lens) >= max_dist - close_enough)
        isExpanding = false;
        isFixed(isLeader) = ~isFixed(isLeader);
    end

    % Done contracting
    if ~isExpanding && (min(super_leader_edge_lens) <= min_dist + close_enough)
        isExpanding = true;
        isFixed(isLeader) = ~isFixed(isLeader);
    end

    % Limit velocity to dx_max
    constrained_dx = dx./rowmag(dx).*min(dx_max, rowmag(dx));
    constrained_dx(isnan(constrained_dx)) = 0;
    x = x + constrained_dx;

    % Constrain to screen
    x(:,1) = bound(x(:,1), window(1), window(1)+window(3));
    x(:,2) = bound(x(:,2), window(2), window(2)+window(4));

    % Reverse fixed node if we hit a wall
    if ( ...
        any(x(:,1)==window(1)) || any(x(:,1)==window(1)+window(3)) || ...
        any(x(:,2)==window(2)) || any(x(:,2)==window(2)+window(4)) ...
    )
        isFixed(isLeader) = ~isFixed(isLeader);
    end

    % Update plot
    debug_box.String=strcat( ...
        "Mode = ", ...
        fi(isExpanding, "Expanding", "Contracting") ...
    );
    p.XData = x(:, 1);
    p.YData = x(:, 2);
    %     q.XData = x(:, 1);
    %     q.YData = x(:, 2);
    %     q.UData = dx(:, 1);
    %     q.VData = dx(:, 2);

    % Update node colors
    p.NodeColor = "black";  % Default blue
    highlight(p, isLeader, NodeColor="blue")
    highlight(p, isFixed, NodeColor="red")
    for id = find(isLeader)'
        highlight(p, id, neighbors(G,id), EdgeColor=fi(isExpanding,'g', 'm'));
    end

    pause(0.05)
end

%% Helper Functions

% Delta-disk graph: get edges of all nodes within certain distance
% I've written better code, but this is passable
function edges = proximity_graph(x, dist, G)
    edge_len = @(x, i, j) norm(x(j,1:2)-x(i,1:2));

    N = length(x);
    edges = cell(0, 1);
    for i = 1:N-1
        for j = i+1:N
            % Skip if we already have an edge
            if exist('G', 'var') && findedge(G, i, j)
                continue
            end
            
            % Check distance for edge
            if edge_len(x, i, j) <= dist
                edges{end+1,1} = [i j]; %#ok
            end
        end
    end
    edges = cell2mat(edges);
end
