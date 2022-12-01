set(0,'DefaultFigureWindowStyle','docked')
rowmag = @(A) sqrt(sum(A.^2,2));  % Magnitude of each row
bound = @(A, lower, upper) min(max(A,lower),upper);

v = VideoWriter('Inchworm_Var_Dists_Sim.mp4', 'MPEG-4');
open(v);

% Lower and upper x and y limits, as well as clearance for walls
xmin = 0;
xmax = 10;
ymin = -3;
ymax = 3;
wall_clearance = 0.5;

% Initial positions
x = [3 0; 4 0; 5 0.2; 6 0; 7 0]; % start in extended formation
N = length(x);

% Agent velocities initialization
dx = zeros(size(x));
dx_max = 0.02;

% Edge Weights
w1 = 100;
w2 = 40;
w3 = 10;

% Constant Distances
d = 1;

% Variable Distances
d24 = 0.4;
d13 = 1.6;
d35 = d13;

% Create edges, weights, and distances arrays
edges =   [1 2; 1 3; 2 3; 2 4; 3 4; 3 5; 4 5];
weights = [w1 ; w3 ; w2 ; w2 ; w2 ; w3 ; w1 ];
dists =   [d  ; d13; d  ; d24; d  ; d35; d  ];

% Create graph object
G = graph(edges(:,1), edges(:,2), weights);

mode = 1; % 1:Push tail(1), -1:Pull to head(5)
leader = 5;
mode_leaders = [5 1];
clearance = 0.2;

% Plot graph
p = plot(G, XData=x(:,1), YData=x(:,2));
highlight(p, [1 2 3 4], [2 3 4 5], 'LineWidth', 3);
highlight(p, [1 2 3], [3 4 5], 'LineStyle', '--');
axis equal
xlim([0 10])
ylim([-3 3])

for t = 1:1000
    leader = mode_leaders(mode);
    dx(:,:) = 0; % Reset dx
    for i = 1:N
        for j = neighbors(G, i)'  % Transpose because Matlab is dumb
            w = G.Edges.Weight(findedge(G, i, j));
            target_dist = dists(findedge(G,i,j));
            dx(i,:) = dx(i,:) + 0.5*abs(w)*(norm(x(j,:)-x(i,:))^2 - target_dist^2)*(x(j,:)-x(i,:));
        end
    end
    dx(leader,:) = 0;

    % Mode switching (check if variable distances are close enough to
    % desired values, and switch leader and values if they are
    if abs(norm(x(2,:)-x(4,:)) - dists(findedge(G,2,4))) <= clearance
        if mode == 1
            mode = 2;
            d24 = 1.9;
            d13 = 1.93;
            d35 = d13;
        else
            mode = 1;
            d24 = 0.4;
            d13 = 1.6;
            d35 = d13;
        end
        dists =   [d; d13; d; d24; d; d35; d];
    end

    % Limit velocity to dx_max
    constrained_dx = dx./rowmag(dx).*min(dx_max, rowmag(dx));
    constrained_dx(isnan(constrained_dx)) = 0;
    x = x + constrained_dx;

    % Constrain to screen
    x(:,1) = bound(x(:,1), xmin, xmax);
    x(:,2) = bound(x(:,2), ymin, ymax);
    p.XData = x(:,1);
    p.YData = x(:,2);

%     if abs(x(1,1) - xmin) <= wall_clearance || ...
%        abs(x(1,1) - xmax) <= wall_clearance || ...
%        abs(x(1,2) - ymin) <= wall_clearance || ...
%        abs(x(1,2) - ymax) <= wall_clearance || ...
%        abs(x(5,1) - xmin) <= wall_clearance || ...
%        abs(x(5,1) - xmax) <= wall_clearance || ...
%        abs(x(5,2) - ymin) <= wall_clearance || ...
%        abs(x(5,2) - ymax) <= wall_clearance
%         mode_leaders([1 2]) = mode_leaders([2 1]);
%     end

    if abs(x(leader,1) - xmin) <= wall_clearance || ...
       abs(x(leader,1) - xmax) <= wall_clearance || ...
       abs(x(leader,2) - ymin) <= wall_clearance || ...
       abs(x(leader,2) - ymax) <= wall_clearance
        mode_leaders([1 2]) = mode_leaders([2 1]);
    end
    frame = getframe(gcf);
    writeVideo(v,frame);

    pause(0.02);
end
close(v);
