rowmag = @(A) sqrt(sum(A.^2,2));  % Magnitude of each row
bound = @(A, lower, upper) min(max(A,lower),upper);

tailpush = true;

if tailpush % push agents together from tail
    x = [1 0; 2 0; 3 0.1; 4 0; 5 0]; % start in extended position
    distances = [0 1 0 0 2.25;
                 1 0 1 0.25 0;
                 0 1 0 1 0;
                 0 0.25 1 0 1;
                 2.25 0 0 1 0];
    leader = 5;
else % pull agents towards head
    x = [1 0; 2 0; 2.5 0.8; 3 0; 4 0]; % start in retracted position
    distances = [0 1 0 0 3;
                 1 0 1 2 0;
                 0 1 0 1 0;
                 0 2 1 0 1;
                 3 0 0 1 0];
    leader = 1;
end

dx = zeros(size(x));
dx_max = 0.02;

mode = 1; % 1:Push from tail(1), -1:Pull to head(5)

N = length(x);

edges = [1 2; 2 3; 3 4; 4 5; 2 4; 1 5];
weights = [3; 3; 3; 3; 2; 2];

G = graph(edges(:,1), edges(:,2), weights);
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

    %     if mode == 1 && norm(x(1,:)-x(2,:)) >= max_dist
    %         mode = -1;
    %     end
    %     if mode == -1 && norm(x(1,:)-x(2,:)) <= min_dist
    %         mode = 1;
    %     end
    %     if mode == 1
    %         dx(1,:) = 0;
    %     end

    % Limit velocity to dx_max
    constrained_dx = dx./rowmag(dx).*min(dx_max, rowmag(dx));
    x = x + constrained_dx;

    % Constrain to screen
    x(:,1) = bound(x(:,1), 0, 6);
    x(:,2) = bound(x(:,2), 0, 6);
    %hold on
    plot(G, XData=x(:,1), YData=x(:,2));
    hold on
    % Velocity vectors for convenience
    quiver(x(:,1), x(:,2), dx(:,1), dx(:,2), 0.3);
    xlim([0 7])
    ylim([0 5])
    hold off
    axis equal

    pause(0.05)
end
