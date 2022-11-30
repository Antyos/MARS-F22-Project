clear;
% set(0,'DefaultFigureWindowStyle','docked')
rowmag = @(A) sqrt(sum(A.^2,2));  % Magnitude of each row
bound = @(A, lower, upper) min(max(A,lower),upper);
edge_len = @(x, i, j) norm(x(j,:)-x(i,:));

window = [0 0 6 6]; % (x, y, w, h)
border = 0.5;

tolerance = 0.1;

min_dist = 0.3;
max_dist = 1.5;
dx_max = 0.02;

x = [1 2; 2 2; 3 3; 3 1];  % Position
dx = zeros(size(x)); % Velocity
N = length(x);  % Num nodes

isFixed = false(N, 1);  % Bool mask of which nodes are fixed
isFixed(1) = true;


edges = [1 2; 2 3; 3 4; 2 4];
weights = ones(length(edges), 1);

isExpanding = 1;

G = graph(edges(:,1), edges(:,2), weights);

% Setup plotting
f = figure(1);
clf(f);
p = plot(G, XData=x(:,1), YData=x(:,2));
debug_box = annotation( ...
    textbox=[0.2, 0.5, 0.3, 0.3],...
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
            % If either node is expanding, use max_dist as target
            % If both nodes are contracting, use min_dist as target
            if isExpanding % any(isExpanding([i, j]))
                target_dist = max_dist;
            else
                target_dist = min_dist;
            end

            dx(i,:) = dx(i,:) + 0.1*abs(w)*(edge_len(x, i, j)^2 - target_dist^2)*(x(j,:)-x(i,:));
        end
    end

    % Mode swapping. The state of the system is fully controlled by E(1,2)

    % Done expanding edge   (1,2)
    if isExpanding && (edge_len(x, 1, 2) >= max_dist - tolerance)
        isExpanding = false;
        isFixed([1,end]) = ~isFixed([1,end]);
%         G.Edges.Weight(1) = 1;
    end
    % Done contracting edge (1,2)
    if ~isExpanding && (edge_len(x, 1, 2) <= min_dist + tolerance)
        isExpanding = true;
        isFixed([1,end]) = ~isFixed([1,end]);
%         G.Edges.Weight(1) = -1;
    end

    % Limit velocity to dx_max
    constrained_dx = dx./rowmag(dx).*min(dx_max, rowmag(dx));
    constrained_dx(isnan(constrained_dx)) = 0;
    x = x + constrained_dx;

    % Constrain to screen
    x(:,1) = bound(x(:,1), window(1), window(1)+window(3));
    x(:,2) = bound(x(:,2), window(2), window(2)+window(4));

    if any(x(:)==6) || any(x(:)==0)
        isFixed([1,end]) = ~isFixed([1,end]);
    end

    % Update plot
    debug_box.String=sprintf( ...
        "Mode=%d\nE(1,2)=%.3f (%d,w=%d)\nE(2,3)=%.3f (%d,w=%d)", ...
        isExpanding,...
        edge_len(x,1,2), isFixed(1), G.Edges.Weight(1), ...
        edge_len(x,2,3), isFixed(3), G.Edges.Weight(2) ...
    );
    p.XData = x(:, 1);
    p.YData = x(:, 2);

    % Update node colors
    p.NodeColor = "#0072BD";  % Default blue
    highlight(p, isFixed, NodeColor="red")
    % Velocity vectors for convenience
    % quiver(x(:,1), x(:,2), dx(:,1), dx(:,2), 0.3);

    pause(0.05)
end
