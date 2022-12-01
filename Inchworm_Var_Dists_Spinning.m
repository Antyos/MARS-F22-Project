%% Set up Robotarium object
set(0,'DefaultFigureWindowStyle','docked')

N = 5;
spacing = 0.3;
initial_conditions = generate_initial_conditions(N, 'Width', 2, 'Height', 1, 'Spacing', spacing);
r = Robotarium('NumberOfRobots', N, 'ShowFigure', true, 'InitialConditions', initial_conditions);

%% Create barrier certificates and dynamics transformers

uni_barrier_cert = create_uni_barrier_certificate_with_boundary();
si_barrier_cert = create_si_barrier_certificate_with_boundary();
si_to_uni_dynamics = create_si_to_uni_dynamics('LinearVelocityGain', 0.4, 'AngularVelocityLimit', pi/2);

%% Go to desired inital positions
% initial_positions = [-1.4 -1.1 -0.8 -0.5 -0.2 -0.8; 0 0 0.1 0 0 -0.1; 0 0 0 0 0 0];
initial_positions = [-1.4 -0.9 -0.48 0.05 0.55; ...
                      0    0    0.15 0    0   ; ...
                      0    0    0    0    0   ];

args = {'PositionError', 0.03, 'RotationError', 50};
init_checker = create_is_initialized(args{:});
controller = create_si_position_controller();

x = r.get_poses();
r.step();

while(~init_checker(x, initial_positions))

    x = r.get_poses();
    dxi = controller(x(1:2, :), initial_positions(1:2, :));

    norms = arrayfun(@(x) norm(dxi(:, x)), 1:N);
    threshold = 5/8*r.max_linear_velocity;
    to_thresh = norms > threshold;
    dxi(:, to_thresh) = threshold*dxi(:, to_thresh)./norms(to_thresh);

    dxi = si_barrier_cert(dxi, x(1:2, :));
    dxu = si_to_uni_dynamics(dxi, x);

    r.set_velocities(1:N, dxu);
    r.step();
end

%% Set up constants and variables for inchworm formation control

% Magnitude of each row
rowmag = @(A) sqrt(sum(A.^2,2));
edge_len = @(x, i, j) norm(x(j,1:2)-x(i,1:2));

% Lower and upper x and y limits, as well as clearance for walls
wall_clearance = 0.1;

% Agent velocities initialization
dx = zeros(N,2);
dx_max = 3/4*r.max_linear_velocity;

% Edge Weights
w1 = 100;
w2 = 40;
w3 = 10;

% Constant Distances
d = 0.5;

% Variable Distances
d24 = 0.30;
d13 = 0.81;
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
close_enough = 0.2;

formation_control_gain = 10;

iterations = 5000;

x = r.get_poses()';

% Plot graph
p = plot(G, 'XData', x(:,1), 'YData', x(:,2));
highlight(p, [1 2 3 4], [2 3 4 5]);

r.step()

%% Inchworm formation control

for t = 0:iterations
    % Retrieve the most recent poses from the Robotarium.  The time delay is
    % approximately 0.033 seconds
    x = r.get_poses()';
    
    % Swap mode leaders if we get too close to a wall
    if  any(x(:,1)<=r.boundaries(1)+wall_clearance) || ...
            any(x(1:2,1)>=r.boundaries(2)-wall_clearance) || ...
            any(x(1:2,2)<=r.boundaries(3)+wall_clearance) || ...
            any(x(1:2,2)>=r.boundaries(4)-wall_clearance)
        mode_leaders([1 2]) = mode_leaders([2 1]);
    end
    
    % Get leader agent and reset dx
    leader = mode_leaders(mode);
    dx(:,:) = 0;

    for i = 1:N
        if i ~= leader
            for j = neighbors(G, i)'
                w = G.Edges.Weight(findedge(G, i, j));
                target_dist = dists(findedge(G,i,j));
                dx(i,:) = dx(i,:) + formation_control_gain*w*(edge_len(x(:,1:2), i, j)^2 - target_dist^2)*(x(j,1:2)-x(i,1:2));
            end
        end
    end

    % Mode switching (check if variable distances are close enough to
    % desired values, and switch leader and values if they are
    if abs(edge_len(x,2,4) - dists(findedge(G,2,4))) <= close_enough
        if mode == 1
            mode = 2;
            d24 = 0.95;
            d13 = 0.99;
            d35 = d13;
        else
            mode = 1;
            d24 = 0.30;
            d13 = 0.81;
            d35 = d13;
        end
        dists =   [d; d13; d; d24; d; d35; d];
    end

    % Avoid actuator errors
    % To avoid errors, we need to threshold dx

    dxi = dx./rowmag(dx).*min(dx_max, rowmag(dx));
    dxi(isnan(dxi)) = 0;

%     norms = arrayfun(@(x) norm(dx(:, x)), 1:N);
%     threshold = 5/8*r.max_linear_velocity;
%     to_thresh = norms > threshold;
%     dx(:, to_thresh) = threshold*dx(:, to_thresh)./norms(to_thresh);

    % Transform the single-integrator dynamics to unicycle dynamics using a provided utility function
    
    dxu = si_to_uni_dynamics(dxi', x');
    dxu = uni_barrier_cert(dxu, x');

%     dx = si_to_uni_dyn(dx, x);
%     dx = uni_barrier_cert(dx, x);

    % Set velocities of agents 1:N
    r.set_velocities(1:N, dxu);

    p.XData = x(:, 1);
    p.YData = x(:, 2);

    % Send the previously set velocities to the agents.  This function must be called!
    r.step();

end

r.debug();
