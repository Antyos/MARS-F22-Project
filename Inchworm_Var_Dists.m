%% Set up Robotarium object
set(0,'DefaultFigureWindowStyle','docked')

N = 6;
initial_conditions = generate_initial_conditions(N, 'Width', 2, 'Height', 1, 'Spacing', 0.5);
r = Robotarium('NumberOfRobots', N, 'ShowFigure', true, 'InitialConditions', initial_conditions);

%% Create barrier certificates and dynamics transformers

uni_barrier_cert = create_uni_barrier_certificate_with_boundary();
si_barrier_cert = create_si_barrier_certificate_with_boundary();
si_to_uni_dynamics = create_si_to_uni_dynamics('LinearVelocityGain', 0.4, 'AngularVelocityLimit', pi/2);

%% Go to desired inital positions
initial_positions = [-1.4 -1.1 -0.8 -0.5 -0.2 -0.8; 0 0 0.1 0 0 -0.1; 0 0 0 0 0 0];

args = {'PositionError', 0.03, 'RotationError', 50};
init_checker = create_is_initialized(args{:});
controller = create_si_position_controller();

x=r.get_poses();
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

% Lower and upper x and y limits, as well as clearance for walls
wall_clearance = 0.1;

% Agent velocities initialization
dx = zeros(N,2);
dx_max = 3/4*r.max_linear_velocity;

% Edge Weights
w1 = 80;
w2 = 40;
w3 = 10;

% Constant Distances
d = 1;
% Variable Distances
d14 = 1.4;
d25 = 1.4;
d36 = 1.96;

% Create edges, weights, and distances arrays
edges =   [1 2; 1 4; 2 3; 2 5; 2 6; 3 4; 3 6; 4 5; 4 6];
weights = [w1 ; w2 ; w3 ; w2 ; w3 ; w3 ; w3 ; w1 ; w3 ];
dists =   [d  ; d14; d  ; d25; d  ; d  ; d36; d  ; d  ];

% Create graph object
G = graph(edges(:,1), edges(:,2), weights);

mode = 1; % 1:Push tail, 2:Pull to head
leader = 5;
mode_leaders = [5 1];
close_enough = 0.05;

formation_control_gain = 10;

iterations = 5000;

x = r.get_poses()';
r.step()

%% Inchworm formation control

for t = 0:iterations
    % Retrieve the most recent poses from the Robotarium.  The time delay is
    % approximately 0.033 seconds
    x = r.get_poses()';
    
    % Swap mode leaders if we get too close to a wall
    if ( ...
        any(x(:,1)<=r.boundaries(1)+wall_clearance) || ...
        any(x(:,1)>=r.boundaries(2)-wall_clearance) || ...
        any(x(:,2)<=r.boundaries(3)+wall_clearance) || ...
        any(x(:,2)>=r.boundaries(4)-wall_clearance) ...
    )
        mode_leaders([1 2]) = mode_leaders([2 1]);
    end
    
    % Get leader agent and reset dx
    leader = mode_leaders(mode);
    dx(:,:) = 0;

    for i = 1:N
        for j = neighbors(G, i)'
            w = G.Edges.Weight(findedge(G, i, j));
            target_dist = dists(findedge(G,i,j));
            dx(i,:) = dx(i,:) + formation_control_gain*abs(w)*(norm(x(j,1:2)-x(i,1:2))^2 - target_dist^2)*(x(j,1:2)-x(i,1:2));
        end
    end
    dx(leader,:) = 0;

    % Mode switching (check if variable distances are close enough to
    % desired values, and switch leader and values if they are
    if abs(norm(x(1,1:2)-x(4,1:2)) - dists(findedge(G,1,4))) <= close_enough && ...
       abs(norm(x(2,1:2)-x(5,1:2)) - dists(findedge(G,2,5))) <= close_enough && ...
       abs(norm(x(3,1:2)-x(6,1:2)) - dists(findedge(G,3,6))) <= close_enough
        if mode == 1
            mode = 2;
            d14 = 2.9;
            d25 = 2.9;
            d36 = 0.40;
        else
            mode = 1;
            d14 = 1.40;
            d25 = 1.40;
            d36 = 1.96;
        end
        dists = [d; d14; d; d25; d; d; d36; d; d];
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

    % Send the previously set velocities to the agents.  This function must be called!
    r.step();

end

r.debug();
