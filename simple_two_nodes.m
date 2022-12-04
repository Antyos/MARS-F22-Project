% Basic simulation that tests two nodes with mode swapping

rowmag = @(A) sqrt(sum(A.^2,2));  % Magnitude of each row
bound = @(A, lower, upper) min(max(A,lower),upper);

tailpush = true;

x = [1 2; 2 2];

min_dist = 0.5;
max_dist = 1;

dx = zeros(size(x));
dx_max = 0.02;

N = length(x);

edges = [1 2];
weights = [1];

G = graph(edges(:,1), edges(:,2), weights);

mode = 1; % 1:Push from tail(1), -1:Pull to head(5)
clearance = 0.1;

for t = 1:500
    dx(:,:) = 0; % Reset dx
    % Consensus
    for i = 1:N
        for j = neighbors(G, i)'  % Transpose because Matlab is dumb
            w = G.Edges.Weight(findedge(G, i, j));
            if mode == 1
                target_dist = max_dist;
            else
                target_dist = min_dist;
            end
            dx(i,:) = dx(i,:) + 0.1*abs(w)*(norm(x(j,:)-x(i,:))^2 - target_dist^2)*(x(j,:)-x(i,:));
        end
    end

    % Mode swapping
    if mode == 1 && (norm(x(1,:)-x(2,:)) >= max_dist - clearance)
        mode = 2;
    end
    if mode == 2 && (norm(x(1,:)-x(2,:)) <= min_dist + clearance)
        mode = 1;
    end

    % Leader is static here
    dx(mode,:) = 0;

    % Limit velocity to dx_max
    constrained_dx = dx./rowmag(dx).*min(dx_max, rowmag(dx));
    constrained_dx(isnan(constrained_dx)) = 0;
    x = x + constrained_dx;

    % Constrain to screen
    x(:,1) = bound(x(:,1), 0, 6);
    x(:,2) = bound(x(:,2), 1, 3);
    %hold on
    plot(G, XData=x(:,1), YData=x(:,2));
    axis equal
    hold on
    % Velocity vectors for convenience
    quiver(x(:,1), x(:,2), dx(:,1), dx(:,2), 0.3);
    xlim([0 7])
    ylim([1 3])
    hold off

    pause(0.01);
end
