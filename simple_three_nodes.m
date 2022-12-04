%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Simple simulation of three nodes using the target distance-swapping
% controller. Edges must be manually defined.
%
% Author: Andrew Glick
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear;
% set(0,'DefaultFigureWindowStyle','docked')
rowmag = @(A) sqrt(sum(A.^2,2));  % Magnitude of each row
bound = @(A, lower, upper) min(max(A,lower),upper);
edge_len = @(x, i, j) norm(x(j,:)-x(i,:));

window = [0 0 3 2]; % (x, y, w, h)
border = 0.5;

tolerance = 0.05;

min_dist = 0.5;
max_dist = 1;
dx_max = 0.02;

x = [1 1; 2 1; 2.5 1];  % Position
dx = zeros(size(x)); % Velocity
N = length(x);  % Num nodes

isLeader= false(N, 1);
isLeader([1, end]) = true;
isFixed = false(N, 1);  % Bool mask of which nodes are fixed
isFixed(1) = true;


edges = [1 2; 2 3];
weights = ones(length(edges), 1);

isExpanding = 1;

G = graph(edges(:,1), edges(:,2), weights);

% Setup plotting
f = figure(1);
clf(f);
p = plot(G, XData=x(:,1), YData=x(:,2));
hold on
% Velocity vectors for convenience
q = quiver(x(:,1), x(:,2), dx(:,1), dx(:,2), 0.3);
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
for t = 1:1000
    % Reset dx
    dx(:,:) = 0;

    % Consensus
    for i = 1:N
        % Skip leader; it's static
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

            dx(i,:) = dx(i,:) + 0.1*abs(w)*(edge_len(x, i, j)^2 - target_dist^2)*(x(j,:)-x(i,:));
        end
    end

    %%%%%%%%
    % Mode swapping. The state of the system is fully controlled by E(1,2)
    %%%%%%%%

    % Done expanding edge   (1,2)
    if isExpanding && (edge_len(x, 1, 2) >= max_dist - tolerance)
        isExpanding = false;
        isFixed(isLeader) = ~isFixed(isLeader);
    end

    % Done contracting edge (1,2)
    if ~isExpanding && (edge_len(x, 1, 2) <= min_dist + tolerance)
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
    debug_box.String=sprintf("Mode=%d", isExpanding);
    p.XData = x(:, 1);
    p.YData = x(:, 2);
    q.XData = x(:, 1);
    q.YData = x(:, 2);
    q.UData = dx(:, 1);
    q.VData = dx(:, 2);

    % Update node colors
    p.NodeColor = "black";  % Default blue
    highlight(p, isLeader, NodeColor="blue")
    highlight(p, isFixed, NodeColor="red")
    for id = find(isLeader)'
        if isExpanding
            c = "green";
        else
            c = "magenta";
        end
        highlight(p, id, neighbors(G,id), EdgeColor=c);
    end

    pause(0.05)
end
