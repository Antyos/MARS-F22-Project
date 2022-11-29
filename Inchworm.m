%% Set up Robotarium object
set(0,'DefaultFigureWindowStyle','docked')

N = 4;
initial_conditions = generate_initial_conditions(N, Width=2, Height=1, Spacing=0.5);
r = Robotarium(NumberOfRobots=N, ShowFigure=true, InitialConditions=initial_conditions);

if ~exist("out", "dir")
    mkdir("out")
end

%% Set up constants for experiment

%Gains for the transformation from single-integrator to unicycle dynamics
formation_control_gain = 10;

% Select the number of iterations for the experiment.  This value is
% arbitrary
iterations = 8000;

% Communication topology for the desired formation.  We need 2 * N - 3 = 9
% edges to ensure that the formation is rigid.
L = [ 2 -1 -1 -1; ...
     -1  3 -1  0; ...
     -1 -1  3 -1; ...
     -1  0 -1  2];
% Mode variable
% 1: extend (formation/consensus with larger defined weights)
% 2: retract (formation/consensus with smaller defined weights)
% 3: retract (consensus with no defined weights)
mode = 1;
    
% Initialize velocity vector for agents.  Each agent expects a 2 x 1
% velocity vector containing the linear and angular velocity, respectively.
dx = zeros(2, N);

%% Grab tools for converting to single-integrator dynamics and ensuring safety 

uni_barrier_cert = create_uni_barrier_certificate_with_boundary();
si_to_uni_dyn = create_si_to_uni_dynamics(LinearVelocityGain=0.5, AngularVelocityLimit=pi/2);

% Iterate for the previously specified number of iterations
for t = 0:iterations
    % Change mode variables
    switch mode
        case 1 
            d = 0.4;
            leader = 1;
            weights = [  0   d 2*d 3*d; ...
                         d   0   d   0; ...
                       2*d   d   0   d; ...
                       3*d   0   d   0];
        case 2
            d = 0.3;
            dd = sqrt(2*(d^2));
            leader = 4;
            weights = [ 0  d  d dd; ...
                        d  0 dd  0; ...
                        d dd  0  d; ...
                       dd  0  d  0];
        case 3
            leader = 4;
    end

    % Weight matrix containing the desired inter-agent distances to achieve a
    % rectuangular formation
    
    
    % Retrieve the most recent poses from the Robotarium.  The time delay is
    % approximately 0.033 seconds
    x = r.get_poses();
    
    %% Algorithm
    
    %This section contains the actual algorithm for formation control!
    
    %Calculate single integrator control inputs using edge-energy consensus
    for i = 1:N
        
        % Initialize velocity to zero for each agent.  This allows us to sum
        % over agent i's neighbors
        dx(:, i) = [0 ; 0];
        
        % Get the topological neighbors of agent i from the communication
        % topology
        if(i ~= leader)
            for j = topological_neighbors(L, i)
                
                % For each neighbor, calculate appropriate formation control term and
                % add it to the total velocity
                
                switch mode
                    case 1
                        dx(:, i) = dx(:, i) + ...
                        formation_control_gain*(norm(x(1:2, i) - x(1:2, j))^2 - weights(i, j)^2) ... 
                        *(x(1:2, j) - x(1:2, i));
                    case 2
                        dx(:, i) = dx(:, i) + ...
                        formation_control_gain*(norm(x(1:2, i) - x(1:2, j))^2 - weights(i, j)^2) ... 
                        *(x(1:2, j) - x(1:2, i));
                    case 3
                        dx(:, i) = dx(:, i) + (x(1:2, j) - x(1:2, i));
                end
            end
        end
    end
    
    %% Avoid actuator errors
    
    % To avoid errors, we need to threshold dx
    norms = arrayfun(@(x) norm(dx(:, x)), 1:N);
    threshold = 3/4*r.max_linear_velocity;
    to_thresh = norms > threshold;
    dx(:, to_thresh) = threshold*dx(:, to_thresh)./norms(to_thresh);
    
    % Transform the single-integrator dynamics to unicycle dynamics using a provided utility function
    dx = si_to_uni_dyn(dx, x);  
    dx = uni_barrier_cert(dx, x);
    
    % Set velocities of agents 1:N
    r.set_velocities(1:N, dx);
    
    % Send the previously set velocities to the agents.  This function must be called!
    r.step();

    %% Switch mode (time/iteration based)

    if(mod(t,2000) == 0)
        mode = 1;
    elseif(mod(t,1000) == 0)
        mode = 2;
    end

end

% We can call this function to debug our experiment!  Fix all the errors
% before submitting to maximize the chance that your experiment runs
% successfully.
r.debug();
