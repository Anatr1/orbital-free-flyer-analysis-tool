%% Set up the initial conditions:

% Specify the orbit parameters, such as the radius and gravitational parameter of the circular orbit.
mu = 398600; % Gravitational parameter of Earth (km^3/s^2)
a = 6871; % Radius of the circular orbit (km)
n = sqrt(mu/a^3); % Mean motion (rad/s)
% Initialize variables
deltaVTotal = 0; % Total Delta V used
reachedTarget = false; % Objective reached flag

% Define the initial positions and velocities of the target (chief) and the chaser (deputy) spacecraft.
% Initial positions and velocities
x0 = 10; % Initial x position of deputy (km)
y0 = 0; % Initial y position of deputy (km)
z0 = 0; % Initial z position of deputy (km)
vx0 = 0; % Initial x velocity of deputy (km/s)
vy0 = 7.5; % Initial y velocity of deputy (km/s)
vz0 = 0; % Initial z velocity of deputy (km/s)

%% Initialize the graphics:

% Set up a MATLAB figure or GUI to visualize the relative motion of the spacecraft.
% Create plots or animations to display the positions and trajectories of the chief and deputy.
% Set up the figure for visualization
figure;
axis equal;
grid on;
xlabel('x (km)');
ylabel('y (km)');
zlabel('z (km)');
title('HCW Equations Simulation');

% Plot the chief's orbit
theta = linspace(0, 2*pi, 100);
chiefOrbitX = a * cos(theta);
chiefOrbitY = a * sin(theta);
chiefOrbitZ = zeros(size(theta));
plot3(chiefOrbitX, chiefOrbitY, chiefOrbitZ, 'r--');
hold on;


%% Simulation loop

% Enter the simulation loop:
% While the objective of reaching the target is not achieved, continue the simulation loop.
while ~reachedTarget
    % Propagate HCW equations
    dt = 0.1; % Time step (s)
    dxdt = vx0;
    dydt = vy0;
    dzdt = vz0;
    dvxdt = -2 * n * dzdt - 3 * n^2 * x0;
    dvydt = 2 * n * dzdt - n^2 * y0;
    dvzdt = -n^2 * z0;
    
    % Within each iteration, propagate the HCW equations to update the positions and velocities of the deputy spacecraft.
    % Update positions and velocities
    x0 = x0 + dxdt * dt;
    y0 = y0 + dydt * dt;
    z0 = z0 + dzdt * dt;
    vx0 = vx0 + dvxdt * dt;
    vy0 = vy0 + dvydt * dt;
    vz0 = vz0 + dvzdt * dt;
    
    % Check termination condition
    distance = sqrt(x0^2 + y0^2 + z0^2)
    if distance <= 0.1 % Example termination condition (within 100 meters of target)
        reachedTarget = true;
    end

    % Plot the deputy's orbit
    deputyOrbitX = x0 + a * cos(theta);
    deputyOrbitY = y0 + a * sin(theta);
    deputyOrbitZ = z0 * ones(size(theta));
    
    % Update visualization
    plot3(x0, y0, z0, 'b.'); % Plot deputy position
    plot3(0, 0, 0, 'ro'); % Plot target position
    plot3(deputyOrbitX, deputyOrbitY, deputyOrbitZ, 'b--'); 
    drawnow;

    
    % Display current Delta V used
    disp(['Current Delta V used: ' num2str(deltaVTotal) ' km/s']);
    
    % Receive Delta V control input from keyboard
    deltaVInput = input('Enter Delta V control input (km/s): ');
    
    % Update deputy velocity with control input
    vx0 = vx0 + deltaVInput;
    
    % Accumulate total Delta V used
    deltaVTotal = deltaVTotal + abs(deltaVInput);
end

% Display final results
% Provide a summary of the total Delta v used to reach the target.
disp(['Total Delta V used: ' num2str(deltaVTotal) ' km/s']);
disp('Objective reached!');









