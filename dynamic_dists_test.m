set(0,'DefaultFigureWindowStyle','docked')
rowmag = @(A) sqrt(sum(A.^2,2));  % Magnitude of each row
bound = @(A, lower, upper) min(max(A,lower),upper);

x = [1 0; 2 0; 3 0.1; 4 0; 5 0]; % start in extended position
dx = zeros(size(x));
dx_max = 0.02;

N = length(x);

edges = [1 2; 2 3; 3 4; 4 5; 2 4; 1 5];
weights = [20; 20; 20; 20; 2; 2];
G = graph(edges(:,1), edges(:,2), weights);

distances = [0    1    0 0    2.25;
             1    0    1 0.25 0;
             0    1    0 1    0;
             0    0.25 1 0    1;
             2.25 0    0 1    0];

mode = 1; % 1:Push from tail(1), -1:Pull to head(5)
leader = 5;
clearance = 0.25;

for t = 1:1000
    dx(:,:) = 0; % Reset dx
    for i = 1:N
        if i ~= leader
            for j = neighbors(G, i)'  % Transpose because Matlab is dumb
                w = G.Edges.Weight(findedge(G, i, j));
                target_dist = distances(i,j);
                dx(i,:) = dx(i,:) + 0.01*abs(w)*(norm(x(j,:)-x(i,:))^2 - target_dist^2)*(x(j,:)-x(i,:));
            end
        end
    end

    if abs(norm(x(1,:)-x(5,:)) - distances(1,5)) <= clearance && ...
       abs(norm(x(2,:)-x(4,:)) - distances(2,4)) <= clearance
        if mode == 1
            mode = -1;
            leader = 1;
            distances = [0 1 0 0 3;
                         1 0 1 2 0;
                         0 1 0 1 0;
                         0 2 1 0 1;
                         3 0 0 1 0];
        else
            mode = 1;
            leader = 5;
            distances = [0    1    0 0    2.25;
                         1    0    1 0.25 0;
                         0    1    0 1    0;
                         0    0.25 1 0    1;
                         2.25 0    0 1    0];
        end
    end

    % Limit velocity to dx_max
    constrained_dx = dx./rowmag(dx).*min(dx_max, rowmag(dx));
    constrained_dx(isnan(constrained_dx)) = 0;
    x = x + constrained_dx;

    % Constrain to screen
    x(:,1) = bound(x(:,1), 0, 10);
    x(:,2) = bound(x(:,2), 0, 6);
    %hold on
    p = plot(G, XData=x(:,1), YData=x(:,2));
    highlight(p, [1 2 3 4], [2 3 4 5]);
    hold on
    % Velocity vectors for convenience
    quiver(x(:,1), x(:,2), dx(:,1), dx(:,2), 0.3);
    xlim([0 10])
    ylim([0 5])
    hold off
    axis equal

    pause(0.05)
end
