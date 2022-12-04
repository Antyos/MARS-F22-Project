%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Robotarium simulation of N nodes using the target distance-swapping
% controller. Set number of robots with `N`. Edges are calculated using a
% delta-disk proximity model.
%
% Author: Andrew Glick
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

rowmag = @(A) sqrt(sum(A.^2,2));  % Magnitude of each row
bound = @(A, lower, upper) min(max(A,lower),upper);
edge_len = @(x, i, j) norm(x(j,1:2)-x(i,1:2));
% Inline conditional: https://stackoverflow.com/a/32253460
fi = @(varargin)varargin{length(varargin)-varargin{1}};

N = 5;  % Number of robots

initial_conditions = generate_initial_conditions(N, "Width", 3, "Height", 1.8, "Spacing", 0.3);
r = Robotarium("NumberOfRobots", N, "ShowFigure", true, "InitialConditions", initial_conditions);

%% Set up constants for experiment

%Gains for the transformation from single-integrator to unicycle dynamics
formation_control_gain = 10;

% Select the number of iterations for the experiment.  This value is
% arbitrary
iterations = 2000;

% Single-integrator -> unicycle dynamics mapping
si_to_uni_dyn = create_si_to_uni_dynamics('LinearVelocityGain', 0.8);
% Single-integrator barrier certificates
uni_barrier_cert = create_uni_barrier_certificate_with_boundary();
% Single-integrator position controller
leader_controller = create_si_position_controller('XVelocityGain', 0.8, 'YVelocityGain', 0.8, 'VelocityMagnitudeLimit', 0.08);

% Font/Marker sizes for Robotarium environment
MARKER_SIZE = determine_marker_size(r, 0.1);
FONT_SIZE = determine_font_size(r, 0.05);

close_enough = 0.05;  % Tolerance for getting close to min/max bounds
wall_border = 0.1;  % Distance to wall border for collisions

min_dist = 0.3;  % Min distance to maintain
max_dist = 1.2;  % Max distance to maintain
dx_max = 3/4*r.max_linear_velocity;   % Max speed

dx = zeros(N,2); % Velocity

% Initial position
x = r.get_poses()';

% Bool mask of leader nodes
isLeader = false(N, 1);
isLeader([1, end]) = true;

% Bool mask of which nodes are fixed. Note, only leader nodes can be fixed
isFixed = false(N, 1);
isFixed(1) = true;

edges = proximity_graph(x, max_dist);
[num_edges, ~] = size(edges);
weights = ones(num_edges, 1);

isExpanding = true;
super_leader = 1;  % Node responsible for controlling expansion

G = graph(edges(:,1), edges(:,2), weights, N);

% Setup plotting
p = plot(G, "XData", x(:,1), "YData", x(:,2), ...
    "LineWidth",3, ...
    "EdgeColor", "black", ...
    "MarkerSize", MARKER_SIZE, ...
    "EdgeFontSize", FONT_SIZE ...
);
mode_text = text(-1.5, 0.9, "Mode: ", "FontSize", FONT_SIZE);
% hold on
% % Velocity vectors for convenience
% q = quiver(x(:,1), x(:,2), dx(:,1), dx(:,2), 0.3);
% hold off

r.step()

% Run simulation
for t = 1:iterations
    % Retrieve the most recent poses from the Robotarium.  The time delay is
    % approximately 0.033 seconds
    x = r.get_poses()';
    
    % Update graph
    new_edges = proximity_graph(x, max_dist/2, G);
    if ~isempty(new_edges)
        G = addedge(G, new_edges(:,1), new_edges(:,2), length(new_edges));
        % Replot
        delete(p);
        p = plot(G, "XData", x(:,1), "YData", x(:,2), ...
            "LineWidth",3, ...
            "EdgeColor", "black", ...
            "MarkerSize", MARKER_SIZE, ...
            "EdgeFontSize", FONT_SIZE ...
        );
    end

    % Swap fixed node if we hit a wall
    if ( ...
        any(x(:,1)<=r.boundaries(1)+wall_border) || ...
        any(x(:,1)>=r.boundaries(2)-wall_border) || ...
        any(x(:,2)<=r.boundaries(3)+wall_border) || ...
        any(x(:,2)>=r.boundaries(4)-wall_border) ...
    )
        isFixed(isLeader) = ~isFixed(isLeader);
    end

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

            dx(i,:) = dx(i,:) + formation_control_gain*w*(edge_len(x(:, 1:2), i, j)^2 - target_dist^2)*(x(j,1:2)-x(i,1:2));
        end
    end

    %% Mode swapping. The state of the system is fully controlled by E(1,2)

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

    %% Avoid actuator errors
    % Limit velocity to dx_max
    dxi = dx./rowmag(dx).*min(dx_max, rowmag(dx));
    dxi(isnan(dxi)) = 0;

    % Use barrier certificate and convert to unicycle dynamics
    dxu = si_to_uni_dyn(dxi', x');
    dxu = uni_barrier_cert(dxu, x');

    %Set velocities
    r.set_velocities(1:N, dxu);

    %% Update plot
    p.XData = x(:, 1);
    p.YData = x(:, 2);
%     q.XData = x(:, 1);
%     q.YData = x(:, 2);
%     q.UData = dx(:, 1);
%     q.VData = dx(:, 2);
% 
    mode_text.String = fi(isExpanding, "Mode: Expanding", "Mode: Contracting");

    % Label edges with their length
%     edge_lengths = arrayfun(@(row) edge_len(x, G.Edges.EndNodes(row, 1), G.Edges.EndNodes(row, 2)), 1:G.numedges);
%     labeledge(p, 1:length(edge_lengths), arrayfun(@(s) num2str(s, 2), edge_lengths, 'UniformOutput', false))

    % Update node colors
    p.NodeColor = "black";
    highlight(p, isLeader, "NodeColor", "blue")
    highlight(p, isFixed, "NodeColor", "red")
    for i = find(isLeader)'
        highlight(p, i, neighbors(G,i), "EdgeColor", fi(isExpanding, 'g', 'm'));
    end

    r.step()
end

r.debug()

%% Helper Functions

% Delta-disk graph: get edges of all nodes within certain distance
function edges = proximity_graph(x, dist, G)
    edge_len = @(x, i, j) norm(x(j,1:2)-x(i,1:2));

    [N,~] = size(x);
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


% Marker Size Helper Function to scale size with figure window
% Input: robotarium instance, desired size of the marker in meters
function marker_size = determine_marker_size(robotarium_instance, marker_size_meters)
    % Get the size of the robotarium figure window in pixels
    curunits = get(robotarium_instance.figure_handle, 'Units');
    set(robotarium_instance.figure_handle, 'Units', 'Points');
    cursize = get(robotarium_instance.figure_handle, 'Position');
    set(robotarium_instance.figure_handle, 'Units', curunits);
    
    % Determine the ratio of the robot size to the x-axis (the axis are
    % normalized so you could do this with y and figure height as well).
    marker_ratio = (marker_size_meters)/(robotarium_instance.boundaries(2) -...
        robotarium_instance.boundaries(1));
    
    % Determine the marker size in points so it fits the window. cursize(3) is
    % the width of the figure window in pixels. (the axis are
    % normalized so you could do this with y and figure height as well).
    marker_size = cursize(3) * marker_ratio;
end

% Font Size Helper Function to scale size with figure window
% Input: robotarium instance, desired height of the font in meters
function font_size = determine_font_size(robotarium_instance, font_height_meters)  
    % Get the size of the robotarium figure window in point units
    curunits = get(robotarium_instance.figure_handle, 'Units');
    set(robotarium_instance.figure_handle, 'Units', 'Pixels');
    cursize = get(robotarium_instance.figure_handle, 'Position');
    set(robotarium_instance.figure_handle, 'Units', curunits);
    % Determine the ratio of the font height to the y-axis
    font_ratio = (font_height_meters)/(robotarium_instance.boundaries(4) -...
        robotarium_instance.boundaries(3));
    % Determine the font size in points so it fits the window. cursize(4) is
    % the hight of the figure window in points.
    font_size = cursize(4) * font_ratio;
end
