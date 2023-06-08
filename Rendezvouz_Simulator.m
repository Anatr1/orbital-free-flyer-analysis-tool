%% RENDEZVOUS SIMULATOR
%
% Federico Mustich, Lorenzo Porpiglia, Gaetana Gaia Spanò, Vincenzo Trentacapilli
%
% This script simulates the rendezvous between a chief and a deputy in a
% circular orbit around the Earth. The deputy is initially setted at a
% random position and velocity. The chief is considered fixed at the origin
% of our CCS. The deputy's trajectory is computed using the Clohessy-Wiltshire
% equations. The user can control the deputy's trajectory using the keyboard.
% The goal is to reach the chief's position (with a tolerance of 0.5 m) with a
% safe velocity (with a tolerance of 0.01 m/s). The user can also
% pause the simulation and exit it or adjust the time rate of the simulation.
%
% COMMANDS:
%   - W: increase the deputy's velocity along the x-axis (Prograde acceleration)
%   - S: decrease the deputy's velocity along the x-axis (Retrograde acceleration)
%   - A: increase the deputy's velocity along the y-axis (Normal acceleration)
%   - D: decrease the deputy's velocity along the y-axis (Anti-normal acceleration)
%   - Q: increase the deputy's velocity along the z-axis (Radial acceleration)
%   - E: decrease the deputy's velocity along the z-axis (Anti-radial acceleration)
%   - P: pause the simulation
%   - T: increase the time rate of the simulation
%   - G: decrease the time rate of the simulation
%   - SPACE: pause the simulation
%   - X: exit the simulation


clc
clear all


%% GLOBAL VARIABLES

global deltaV;
global deltaV_x;
global deltaV_y;
global deltaV_z;
global deltaV_total;
global impulse;
global time_rate;
global paused;
global exit;


%% CONSTANTS

g=9.81; % [m/s2]
mu=3.986005e14; % [m3/s2] 
earth_radius=6371; % [km]
height=500; % [km] Orbit's altitude
R=earth_radius+height; % [km]


%%  INITIAL CONDITIONS

t0=0; % [s]

% Chief is considered fixed at the origin our CCS
Chief=[0;0;0]; 

% Deputy is setted at a random position and velocity
pos_start=[rand*10*randi([-1 1]) rand*10*randi([-1 1]) rand*10*randi([-1 1])]'; % [m]
vel_start=[rand/100*randi([-1 1]) rand/100*randi([-1 1]) rand/100*randi([-1 1])]'; % [m/s] 
time_limit=1500; % [s] computation limit for the trajectory
start_pos_vel=[pos_start; vel_start]; % [m] [m/s]
mean_motion=sqrt(mu/(R*1000)^3); % [rad/s]
orbital_period=sqrt((4*pi^2*(R*1000)^3)/mu); % [s]

% State matrix for the Clohessy-Wiltshire equations
state_matrix = [zeros(3),eye(3);[3*(mean_motion^2) 0 0 0 2*mean_motion 0; 0 0 0 -2*mean_motion 0 0; 0 0 -mean_motion^2 0 0 0]];


%% SIMULATION VARIABLES

[orbit]=ComputeOrbit(time_limit,t0,start_pos_vel,state_matrix);

current_pos_vel=start_pos_vel; % current initial conditions
planned_orbit=[0; 0; 0; 0; 0; 0]; 
deltaV=0; % [m/s]
deltaV_x=0; % [m/s]
deltaV_y=0; % [m/s]
deltaV_z=0; % [m/s]
deltaV_total=0; % [m/s]
t=0; % [s]
impulse = 0.0005; % [m/s] impulse for the maneuver
tolerance_pos = 0.5; % [m] position tolerance for the rendezvous
tolerance_vel = 0.01; % [m/s] velocity tolerance for the rendezvous
time_rate = 0.1; % time rate of the simulation
fig=figure('WindowState', 'maximized');
% Set the KeyPressFcn property of the figure
set(fig, 'KeyPressFcn', @keyPressCallback);

% Objective reached flag
reachedTarget = false; 

% Pause flag
paused = false; 

% Exit flag
exit = false;


%% UI ELEMENTS

uicontrol('Parent',figure(1),'Style','text','Position',[10 50 100 70],'String', "ΔV [m/s]:");
total_delta_v=uicontrol('Parent',figure(1),'Style','text','Position',[10 20 100 70],'String', "0");

uicontrol('Parent',figure(1),'Style','text','Position',[10 230 100 100],'String', "Deputy Position:");
pos=uicontrol('Parent',figure(1),'Style','text','Position',[10 220 100 70],'String', current_pos_vel(1:3));

uicontrol('Parent',figure(1),'Style','text','Position',[10 350 130 70],'String', "Distance From Chief [m]:");
dis=uicontrol('Parent',figure(1),'Style','text','Position',[10 310 100 70],'String', EuclideanDistance(current_pos_vel(1:3), Chief));

uicontrol('Parent',figure(1),'Style','text','Position',[10 750 150 70],'String', "Projected Position From Chief [m]:");
proj_pos=uicontrol('Parent',figure(1),'Style','text','Position',[10 710 100 70],'String', planned_orbit(1:3, end));

uicontrol('Parent',figure(1),'Style','text','Position',[10 870 150 70],'String', "Projected Distance From Chief [m]:");
proj_dis=uicontrol('Parent',figure(1),'Style','text','Position',[10 830 100 70],'String', EuclideanDistance(planned_orbit(1:3, end), Chief));

uicontrol('Parent',figure(1),'Style','text','Position',[160 230 100 100],'String', "Deputy Velocity [m/s]:");
vel=uicontrol('Parent',figure(1),'Style','text','Position',[160 220 100 70],'String', current_pos_vel(4:6));

uicontrol('Parent',figure(1),'Style','text','Position',[160 350 130 70],'String', "Deputy Velocity (norm) [m/s]:");
vel_norm=uicontrol('Parent',figure(1),'Style','text','Position',[160 310 100 70],'String', norm(current_pos_vel(4:6)));

uicontrol('Parent',figure(1),'Style','text','Position',[160 750 150 70],'String', "Projected Velocity [m/s]:");
proj_vel=uicontrol('Parent',figure(1),'Style','text','Position',[160 710 100 70],'String', planned_orbit(4:6, end));

uicontrol('Parent',figure(1),'Style','text','Position',[160 870 150 70],'String', "Projected Velocity (norm) [m/s]:");
proj_vel_norm=uicontrol('Parent',figure(1),'Style','text','Position',[160 830 100 70],'String', norm(planned_orbit(4:6, end)));

uicontrol('Parent',figure(1),'Style','text','Position',[10 500 150 70],'String', "Time Factor:");
time_factor=uicontrol('Parent',figure(1),'Style','text','Position',[10 480 100 70],'String', 1/(10*time_rate));

uicontrol('Parent',figure(1),'Style','text','Position',[160 500 150 70],'String', "Time Elapsed [s]:");
time_elapsed=uicontrol('Parent',figure(1),'Style','text','Position',[160 480 100 70],'String', t);

uicontrol('Parent',figure(1),'Style','text','Position',[500 10 1000 70],'String', "W: Prograde acceleration, S: Retrograde acceleration, A: Normal acceleration, D: Anti-normal acceleration, Q: Radial acceleration, E: Anti-radial acceleration, T: Increase time rate, G: Decrease time rate, Space: Pause, X: Exit");

%% MAIN LOOP

while ~reachedTarget
    if ~paused
        if exit
            disp("Simulation terminated by user");
            close all
        end

        % Prepare Delta V 
        deltaV=[deltaV_x; deltaV_y; deltaV_z];

        % Iteratevely update orbit
        planned_orbit=ComputeOrbit(time_limit,t0,current_pos_vel,state_matrix); % Deputy's orbit
        deputy_pos_vel=ComputeOrbit(t,t0,current_pos_vel,state_matrix); % Deputy's position at time t
        t0=t; % Update time
        current_pos_vel=[deputy_pos_vel(1:3,end); deputy_pos_vel(4:6,end)+deltaV]; % Update initial conditions

        % Update UI Elements
        total_delta_v.String=num2str(deltaV_total);
        pos.String=current_pos_vel(1:3);
        dis.String=EuclideanDistance(current_pos_vel(1:3), Chief);
        vel.String=current_pos_vel(4:6);
        vel_norm.String=norm(current_pos_vel(4:6));
        proj_dis.String=EuclideanDistance(planned_orbit(1:3, end), Chief);
        proj_pos.String=planned_orbit(1:3, end);
        proj_vel.String=current_pos_vel(4:6);
        proj_vel_norm.String=norm(planned_orbit(4:6, end));
        time_factor.String=1/(10*time_rate);
        time_elapsed.String=t;

        % Needed to reset Delta V after one impulse
        if norm(deltaV)~=0
            deltaV_x = 0;
            deltaV_y = 0;
            deltaV_z = 0;
        end

        % Update Deputy's orbit and position and plot everything
        PlotOrbit(current_pos_vel,planned_orbit,state_matrix,time_limit,t0);
        plot3(deputy_pos_vel(1,end),deputy_pos_vel(2,end),deputy_pos_vel(3,end),'o', 'MarkerEdgeColor','#D95319','MarkerSize',10,'MarkerFaceColor','#0072BD')
        plot3(Chief(1),Chief(2),Chief(3),'o','MarkerEdgeColor','#D95319','MarkerSize',10,'MarkerFaceColor','#D95319')
        title('ORBITAL RENDEZVOUS SIMULATOR');
        hold off

        % Check if goal distance and velocity are within acceptable range
        if EuclideanDistance(current_pos_vel(1:3), Chief) <= tolerance_pos && norm(current_pos_vel(4:6)) <= tolerance_vel
            disp('RENDEZVOUS COMPLETED')
            break
        end

        % Update Time and Sleep
        t=t+1;
        pause(time_rate)
    else
        pause(0.1)
    end

end

%% UTILITY FUNCTIONS

% Define the keyPressCallback function
function keyPressCallback(src, event)
    global deltaV; 
    global deltaV_x;
    global deltaV_y;
    global deltaV_z;
    global deltaV_total;
    global impulse;
    global time_rate;
    global paused;
    global exit;
    
    switch event.Key
        case 'w'
            % Prograde acceleration
            disp("Prograde acceleration");
            deltaV_x = deltaV_x + impulse;
        case 's'
            % Retrograde acceleration
            disp("Retrograde acceleration");
            deltaV_x = deltaV_x - impulse;
        case 'a'
            % Left acceleration
            disp("Normal acceleration");
            deltaV_y = deltaV_y + impulse;
        case 'd'
            % Right acceleration
            disp("Anti-normal acceleration");
            deltaV_y = deltaV_y - impulse;
        case 'q'
            % Up acceleration
            disp("Radial-out acceleration");
            deltaV_z = deltaV_z + impulse;
        case 'e'
            % Down acceleration
            deltaV_z = deltaV_z - impulse;
            disp("Radial-in acceleration");
        case 't'
            % Speed Up Time
            time_rate = time_rate * 0.5;
            if time_rate < 0.0001
                time_rate = 0.0001;
            end
            disp("Speed Up Time");
        case 'g'
            % Slow Down Time
            time_rate = time_rate * 2;
            if time_rate > 0.1
                time_rate = 0.1;
            end
            disp("Slow Down Time");
        case 'space'
            % Pause
            if ~paused
                disp("Pause");
            else
                disp("Restarted");
            end
            paused = ~paused;
        case 'x'
            % Exit
            disp("Exit");
            exit = true;
    end

    deltaV=[deltaV_x; deltaV_y; deltaV_z];
    deltaV_total=deltaV_total+norm(deltaV);
end

function [orbit] = ComputeOrbit(time_limit,t0,start_pos_vel,state_matrix)
    % This function computes the orbit of the deputy satellite
    % given the initial conditions and the time limit of the simulation.
    % The output is a matrix where each column represents the state vector
    % at a given time step.

    dt = 0.1; % time step
    t = t0:dt:time_limit; % time vector
    orbit = zeros(6,length(t)); % preallocate output matrix
    orbit(:,1) = start_pos_vel; % set initial conditions

    for i = 2:length(t)
        orbit(:,i) = expm(state_matrix*(t(i)-t0))*start_pos_vel; % calculate state vector at time t(i)
    end
end

function PlotOrbit(start_pos_vel,orbit,state_matrix,time_limit,t0)
    % This function plots the orbit of the deputy satellite given
    % the initial conditions and the background plot.

    plot3(orbit(1,:),orbit(2,:),orbit(3,:),'b','LineWidth',1.5) % plot background orbit
    hold on
    grid on
    xlabel('x [m]')
    ylabel('y [m]')
    zlabel('z [m]')
    title('Expected orbit of deputy satellite')
    axis equal

    Xf = expm(state_matrix*(time_limit-t0))*start_pos_vel; % calculate final state vector
    plot3([start_pos_vel(1) Xf(1)],[start_pos_vel(2) Xf(2)],[start_pos_vel(3) Xf(3)],':m','LineWidth',1.5) % plot expected orbit
end


function distance = EuclideanDistance(start_point, end_point)
    distance = sqrt(sum((start_point' - end_point').^2));
end




