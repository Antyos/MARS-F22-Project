rowmag = @(A) sqrt(sum(A.^2,2));  % Magnitude of each row
bound = @(A, lower, upper) min(max(A,lower),upper);

x = [
    0 0;
    1 0;
    2 1;
    3 0;
    5 0;
];

min_dist = 0.2;
max_dist = 1.5;

dx = zeros(size(x));
dx_max = 0.02;

mode = 1; % 1:Repel, -1:Retract

N = length(x);

% edges = table([1 2; 2 3; 3 4], VariableNames="EndNodes");
edges = [1 2; 2 3; 3 4; 2 4; 4 5];
weights = [0.5; 0.3; 1; 0.3; 0.5];

G = graph(edges(:,1), edges(:,2), weights);

for t = 1:1000
    dx(:,:) = 0; % Reset dx
    for i = 1:N
        for j = neighbors(G, i)'  % Transpose because Matlab is dumb
            w = G.Edges.Weight(findedge(G, i, j));
            if mode == 1
                target_dist = min_dist;
            else
                target_dist = max_dist;
            end
            dx(i,:) = dx(i,:) + 0.01*abs(w)*(norm(x(j,:)-x(i,:))^2 - target_dist^2)*(x(j,:)-x(i,:));
        end
    end

    if mode == 1 && norm(x(1,:)-x(2,:)) >= max_dist
        mode = -1;
    end
    if mode == -1 && norm(x(1,:)-x(2,:)) <= min_dist
        mode = 1;
    end
    if mode == 1
        dx(1,:) = 0;
    end

    % Limit velocity to dx_max
    constrained_dx = dx./rowmag(dx).*min(dx_max, rowmag(dx));
    x = x + constrained_dx;

    % Constrain to screen
    x(:,1) = bound(x(:,1), 0, 5);
    x(:,2) = bound(x(:,2), 0, 5);
%     hold on
    plot(G, XData=x(:,1), YData=x(:,2));
    hold on
    % Velocity vectors for convenience
    quiver(x(:,1), x(:,2), dx(:,1), dx(:,2), 0.3);
    xlim([0 5])
    ylim([0 5])
    hold off
    axis equal

    pause(0.02)
end
