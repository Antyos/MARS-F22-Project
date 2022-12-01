rowmag = @(A) sqrt(sum(A.^2,2));  % Magnitude of each row
bound = @(A, lower, upper) min(max(A,lower),upper);
edge_len = @(x, i, j) norm(x(j,:)-x(i,:));

video = VideoWriter("out/old_three_nodes", "MPEG-4");
video.FrameRate = 10;

min_dist = 0.5;
max_dist = 1.5;
dx_max = 0.02;

x = [1 0; 2 0; 3 0];
dx = zeros(size(x));
N = length(x);

tail = 1;
head = 3;

edges = [1 2; 2 3];
weights = [-1 1];

G = graph(edges(:,1), edges(:,2), weights);

% Mode 1: Edge (1,2) expands, others contract 
% Mode 3: Edge (2,3) expands, others contract
mode = 1;

tolerance = 0.1;

open(video);
f = figure(1);
clf(f);
debug_box = annotation( ...
    textbox=[0.2, 0.5, 0.3, 0.3],...
    String=sprintf( ...
        "Mode=%d\nE(1,2)=%.3f\nE(2,3)=%.3f", ...
        mode, edge_len(x, 1, 2), edge_len(x,2,3) ...
    ), ...
    FitBoxToText="on" ...
);
for t = 1:200
    % Reset dx
    dx(:,:) = 0;

    % Consensus
    for i = 1:N
        % Skip leader; it's static
        if i == mode
            continue
        end

        % Neighbors
        for j = neighbors(G, i)'  % Transpose because Matlab is dumb
            w = G.Edges.Weight(findedge(G, i, j));

            dx(i,:) = dx(i,:) + 0.1*w*(edge_len(x, i, j)^2 - min_dist^2)*(x(j,:)-x(i,:));
        end
    end

    % Mode swapping

    % Done expanding edge   (1,2)
    if mode == 1 && (edge_len(x, 1, 2) >= max_dist - tolerance)
        mode = 3;
        G.Edges.Weight(:) = [1 -1];
    end
    % Done contracting edge (1,2)
    if mode == 3 && (edge_len(x, 1, 2) <= min_dist + tolerance)
        mode = 1;
        G.Edges.Weight(:) = [-1 1];
    end
    % Done contracting edge (2,3)
%     if mode == 1 && (edge_len(x, 2, 3) <= min_dist + tolerance)
%         mode = 3;
%         G.Edges.Weight(:) = [1 -1];
%     end
%     % Done expanding edge   (2,3)
%     if mode == 3 && (edge_len(x, 2, 3) >= max_dist - tolerance)
%         mode = 1;
%         G.Edges.Weight(:) = [-1 1];
%     end

    % Limit velocity to dx_max
    constrained_dx = dx./rowmag(dx).*min(dx_max, rowmag(dx));
    constrained_dx(isnan(constrained_dx)) = 0;
    x = x + constrained_dx;

    % Constrain to screen
    x(:,1) = bound(x(:,1), 0, 5);
    x(:,2) = bound(x(:,2), -1, 1);

    set(debug_box, String=sprintf( ...
        "Mode=%d\nE(1,2)=%.3f\nE(2,3)=%.3f", ...
        mode, edge_len(x, 1, 2), edge_len(x,2,3) ...
    ));
    plot(G, XData=x(:,1), YData=x(:,2));
%     hold on
    % Velocity vectors for convenience
%     quiver(x(:,1), x(:,2), dx(:,1), dx(:,2), 0.3);
    xlim([0 5])
    ylim([-1 1])
    hold off
    axis equal

    writeVideo(video, getframe(gcf));
    pause(0.05)
end

close(video);
