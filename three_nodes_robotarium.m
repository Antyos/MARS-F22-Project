% set(0,'DefaultFigureWindowStyle','docked')
rowmag = @(A) sqrt(sum(A.^2,2));  % Magnitude of each row
bound = @(A, lower, upper) min(max(A,lower),upper);
edge_len = @(x, i, j) norm(x(j,:)-x(i,:));

N = 3;  % Number of robots

initial_conditions = generate_initial_conditions(N, "Width", 2, "Height", 1, "Spacing", 0.5);
r = Robotarium("NumberOfRobots", N, "ShowFigure", true, "InitialConditions", initial_conditions);

%% Set up constants for experiment

%Gains for the transformation from single-integrator to unicycle dynamics
formation_control_gain = 10;

% Select the number of iterations for the experiment.  This value is
% arbitrary
iterations = 8000;

close_enough = 0.05;  % Tolerance for getting close to min/max bounds

min_dist = 0.5;  % Min distance to maintain
max_dist = 1;    % Max distance to maintain
dx_max = 3/4*r.max_linear_velocity;   % Max speed

x = [1 1; 2 1; 2.5 1];  % Position
dx = zeros(size(x)); % Velocity


isLeader= false(N, 1);
isLeader([1, end]) = true;
isFixed = false(N, 1);  % Bool mask of which nodes are fixed
isFixed(1) = true;


edges = [1 2; 2 3];
weights = ones(length(edges), 1);

isExpanding = 1;

G = graph(edges(:,1), edges(:,2), weights);

% Setup plotting
x = r.get_poses()';
p = plot(G, XData=x(:,1), YData=x(:,2));
text(-1.5, 0.9, "Mode: ");
mode_text = text(-1.2, 0.9, "");
% hold on
% % Velocity vectors for convenience
% q = quiver(x(:,1), x(:,2), dx(:,1), dx(:,2), 0.3);
% hold off
% debug_box = annotation( ...
%     textbox=[0.2, 0.6, 0.3, 0.3],...
%     String="", ...
%     FitBoxToText="on" ...
% );
% rectangle(Position=window);
% xlim([-border, window(3)+border]+window(1));
% ylim([-border, window(4)+border]+window(2));
% axis equal

r.step()

% Run simulation
for t = 1:iterations
    % Retrieve the most recent poses from the Robotarium.  The time delay is
    % approximately 0.033 seconds
    x = r.get_poses()';

    % Reset dx
    dx(:,:) = 0;

    % Consensus
    for i = 1:N
        % Skip if node is fixed
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

            dx(i,:) = dx(i,:) + abs(w)*(edge_len(x(:, 1:2), i, j)^2 - target_dist^2)*(x(j,1:2)-x(i,1:2));
        end
    end

    %%%%%%%%
    % Mode swapping. The state of the system is fully controlled by E(1,2)
    %%%%%%%%

    % Done expanding edge   (1,2)
    if isExpanding && (edge_len(x, 1, 2) >= max_dist - close_enough*10)
        isExpanding = false;
        isFixed(isLeader) = ~isFixed(isLeader);
    end

    % Done contracting edge (1,2)
    if ~isExpanding && (edge_len(x, 1, 2) <= min_dist + close_enough*10)
        isExpanding = true;
        isFixed(isLeader) = ~isFixed(isLeader);
    end

    %% Avoid actuator errors
    % Limit velocity to dx_max
    dxi = dx./rowmag(dx).*min(dx_max, rowmag(dx));
    dxi(isnan(dxi)) = 0;
        
    % Use barrier certificate and convert to unicycle dynamics
    dxu = si_to_uni_dyn(dxi', x');
    dxu = uni_barrier_cert(dxu, x');

    % Constrain to screen
    x(:,1) = bound(x(:,1), r.boundaries(1), r.boundaries(2));
    x(:,2) = bound(x(:,2), r.boundaries(3), r.boundaries(4));

    %Set velocities
    r.set_velocities(1:N, dxu);

    % Reverse fixed node if we hit a wall
    if ( ...
        any(x(:,1)<=r.boundaries(1)) || any(x(:,1)>=r.boundaries(2)) || ...
        any(x(:,2)<=r.boundaries(3)) || any(x(:,2)>=r.boundaries(4)) ...
    )
        isFixed(isLeader) = ~isFixed(isLeader);
    end

    %% Update plot
%     debug_box.String=sprintf("Mode=%d", isExpanding);
    p.XData = x(:, 1);
    p.YData = x(:, 2);
%     q.XData = x(:, 1);
%     q.YData = x(:, 2);
%     q.UData = dx(:, 1);
%     q.VData = dx(:, 2);
% 
    if isExpanding
        mode_text.String = "Expanding";
    else
        mode_text.String = "Contracting";
    end
    
    % Update node colors
    p.NodeColor = "#0072BD";  % Default blue
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

    r.step()
end

r.debug()