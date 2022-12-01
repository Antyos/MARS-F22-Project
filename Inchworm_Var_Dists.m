%% Set up Robotarium object
set(0,'DefaultFigureWindowStyle','docked')

% v = VideoWriter('Inchworm_Var_Dists.mp4', 'MPEG-4');
% open(v);

% Function for magnitude of each row
rowmag = @(A) sqrt(sum(A.^2,2));
% Function for actual edge lengths
edge_len = @(x, i, j) norm(x(j,1:2)-x(i,1:2));

N = 5;
spacing = 0.25;
initial_conditions = generate_initial_conditions(N, 'Width', 2, 'Height', 1, 'Spacing', spacing);
r = Robotarium('NumberOfRobots', N, 'ShowFigure', true, 'InitialConditions', initial_conditions);

% Define the max linear velocity
dx_max = 3/4*r.max_linear_velocity;

%% Create barrier certificates and dynamics transformers

uni_barrier_cert = create_uni_barrier_certificate_with_boundary();
si_barrier_cert = create_si_barrier_certificate_with_boundary();
si_to_uni_dynamics = create_si_to_uni_dynamics('LinearVelocityGain', 0.4, 'AngularVelocityLimit', pi/2);

%% Go to desired inital positions

% Constant distances
d = 0.6;
d24r = spacing;
d13r = sqrt((d + d24r/2)^2 + d^2 -(d24r/2)^2);
d35r = d13r;
d24e = 2*d - 0.1;
d13e = sqrt((d + d24e/2)^2 + d^2 -(d24e/2)^2);
d35e = d13e;

% Create initial positions
x1o = -1.4;
x2o = x1o + d;
x3o = x2o + d24e/2;
x4o = x3o + d;
x5o = x4o + d;
initial_positions = [x1o x2o x3o        x4o x5o; ...
                     0   0   spacing/2  0   0  ; ...
                     0   0   0          0   0  ];

args = {'PositionError', 0.03, 'RotationError', 50};
init_checker = create_is_initialized(args{:});
controller = create_si_position_controller();

x = r.get_poses();
r.step();

while(~init_checker(x, initial_positions))

    x = r.get_poses();
    dx = controller(x(1:2, :), initial_positions(1:2, :));

    % Avoid actuator errors
    % To avoid errors, we need to threshold dx
    dxi = dx./rowmag(dx).*min(dx_max, rowmag(dx));
    dxi(isnan(dxi)) = 0;

    % Transform the single-integrator dynamics to unicycle dynamics using a provided utility function
    dxu = si_to_uni_dynamics(dxi, x);
    dxu = uni_barrier_cert(dxu, x);

    r.set_velocities(1:N, dxu);
    r.step();

%     frame = getframe(gcf);
%     writeVideo(v,frame);
end

%% Set up constants and variables for inchworm formation control

% Edge Weights
w1 = 100;
w2 = 40;
w3 = 10;

% Variable Distances
d24 = d24r;
d13 = d13r;
d35 = d35r;

% Create edges, weights, and distances arrays
edges =   [1 2; 1 3; 2 3; 2 4; 3 4; 3 5; 4 5];
weights = [w1 ; w3 ; w1 ; w2 ; w1 ; w3 ; w1 ];
dists =   [d  ; d13; d  ; d24; d  ; d35; d  ];

% Create graph object
G = graph(edges(:,1), edges(:,2), weights);

% Leader and mode variables
mode = 1; % 1:Push tail(1), -1:Pull to head(5)
leader = 5;
mode_leaders = [5 1];
close_enough = 0.2;

% Lower and upper x and y limits, as well as clearance for walls
wall_clearance = 0.15;

% Control gain
formation_control_gain = 10;

% Number of iterations
iterations = 5000;

% Get the most recent poses
x = r.get_poses()';

% Agent velocities initialization
dx = zeros(N,2);

% Plot graph
p = plot(G, 'XData', x(:,1), 'YData', x(:,2));
highlight(p, [1 2 3 4], [2 3 4 5], 'LineWidth', 3);
highlight(p, [1 2 3], [3 4 5], 'LineStyle', '--');

r.step()

%% Inchworm formation control

for t = 0:iterations
    % Retrieve the most recent poses from the Robotarium.  The time delay is
    % approximately 0.033 seconds
    x = r.get_poses()';
    
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
    if abs(edge_len(x,2,4) - dists(findedge(G,2,4))) <= close_enough && ...
       abs(edge_len(x,1,3) - dists(findedge(G,1,3))) <= (close_enough - 0.05) && ...
       abs(edge_len(x,3,5) - dists(findedge(G,3,5))) <= (close_enough - 0.05)
        if mode == 1
            mode = 2;
            d24 = d24e;
            d13 = d13e;
            d35 = d35e;
        else
            mode = 1;
            d24 = d24r;
            d13 = d13r;
            d35 = d35r;
        end
        dists =   [d; d13; d; d24; d; d35; d];
    end

    % Avoid actuator errors
    % To avoid errors, we need to threshold dx
    dxi = dx./rowmag(dx).*min(dx_max, rowmag(dx));
    dxi(isnan(dxi)) = 0;

    % Transform the single-integrator dynamics to unicycle dynamics using a provided utility function
    dxu = si_to_uni_dynamics(dxi', x');
    dxu = uni_barrier_cert(dxu, x');

    % Set velocities of agents 1:N
    r.set_velocities(1:N, dxu);

    % Update the plot
    p.XData = x(:, 1);
    p.YData = x(:, 2);

    % Send the previously set velocities to the agents.  This function must be called!
    r.step();

%     frame = getframe(gcf);
%     writeVideo(v,frame);

    % If a leader has gotten too close to a wall, swap the mode leaders
    if abs(x(leader,1) - r.boundaries(1)) <= wall_clearance || ...
       abs(x(leader,1) - r.boundaries(2)) <= wall_clearance || ...
       abs(x(leader,2) - r.boundaries(3)) <= wall_clearance || ...
       abs(x(leader,2) - r.boundaries(4)) <= wall_clearance
        mode_leaders([1 2]) = mode_leaders([2 1]);
    end
end

% close(v);

r.debug();
