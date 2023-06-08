% Federico Mustich, Lorenzo Porpiglia, Gaia Spanò, Vincenzo Trentacapilli

% HCW_IC_MakeFigs plots the spontaneous motion of Deputy respect to Chief
% in a planar orbit (2 dimensions), solving the HCW equations in cartesian 
% coordinates, given user-specified initial conditions on position and velocity.
% Five plots are obtained:

% 1. Motion for various x0 displacements

% 2. Motion for various x0_dot variations

% 3. Motion for various y0_dot variations

% 4. Motion for various x0_dot variations, x0 set  = 50 m

% 5. Motion for various y0_dot variations, x0 set  = 50 m


% The above_mentioned parmenters can be modified by the user, resulting in
% different behaviours by the Deputy, and thus in different curves in the
% graphs.Final time is arbitrarily set to 200 minutes in order to have figures
% as close as possible to the ones on the slides, but it can be chosen by the
% user as well


clear all
close all
clc

h = 500e3; % heigth [m]
R = 6378.137e3; % Earth radius [m]
mu = 3.986e14; % Earth Gravitation Parameter [m^3/s^2]

n = sqrt(mu/(R+h)^3); % mean motion [s^-1]

A = [zeros(3), eye(3);[3*n^2 0 0 0 2*n 0; 0 0 0 -2*n 0 0; 0 0 -n^2 0 0 0]]; % state matrix

X0 = [0 0 0 0 0 0]'; % default initial conditions vector [m; m; m; m/s; m/s; m/s]                     

phi = @(t) expm(A*t); % state transition matrix
X = @(t,X0) phi(t)*X0; 

t = linspace(0,200*60,500);  % [s], total time is set to 200 minutes  
X_t = zeros(length(X0),length(t)); 


val_x = [-5 5 -10 10 -50 50 -100 100 -500 500]; % array of different initial x values [m]
val_x_dot = [-0.5 0.5 -1 1 -5 5 -10 10 -50 50]; % array of different initial x_dot values [m/s]
val_x_dot2 = [0 -0.5 0.5 -1 1 -5 5 -10 10 -50 50]; % array of different initial x_dot values [m/s]
val_y_dot1 = [-0.113 0.113 -0.25 0.25 -0.5 0.5 -1 1 -5 5 -10 10 -50 50]; % array of different initial y_dot values [m/s]
val_y_dot2 = [-24 -12 -2.5 -1.3 -0.176 -0.117 0.058 1.05 2.2 12 24]; % array of different initial y_dot values [m/s]



figure(1)
for i = 1:length(val_x)
X0(1) = val_x(i);
    for j = 1:length(t)
        X_t(:,j) = X(t(j),X0);
    end

plotColors = jet(length(val_x));
plot(X_t(2,:),X_t(1,:),'Color',plotColors(i,:),'LineWidth',1.2)
hold on
grid on
end

axis([-3000 3000 -2000 2000])
yline(0, 'LineWidth',1)
xline(0, 'LineWidth',1)
legend({'-5','5', '-10', '10', '-50', '50', '-100', '100', '-500', '500'},'Location','northeast','NumColumns',2);
title('Motion of Deputy for various x0 displacements')
xlabel('y[m]');
ylabel('x[m]');



figure(2)
X0 = [0 0 0 0 0 0]'; 
for i = 1:length(val_x_dot)
X0(4) = val_x_dot(i);
    for j = 1:length(t)
        X_t(:,j) = X(t(j),X0);
    end

plotColors = jet(length(val_x_dot));
plot(X_t(2,:),X_t(1,:),'Color',plotColors(i,:),'LineWidth',1.2)
hold on
grid on
end

axis([-3000 3000 -2000 2000])
yline(0, 'LineWidth',1)
xline(0, 'LineWidth',1)
legend({'-0.5','0.5', '-1', '1', '-5', '5', '-10', '10', '-50', '50'},'Location','northeast','NumColumns',2);
title('Motion of Deputy for x0-dot variations')
xlabel('y[m]');
ylabel('x[m]');



figure(3)
X0 = [0 0 0 0 0 0]'; 
for i = 1:length(val_y_dot1)
X0(5) = val_y_dot1(i);
    for j = 1:length(t)
        X_t(:,j) = X(t(j),X0);
    end

plotColors = jet(length(val_y_dot1));
plot(X_t(2,:),X_t(1,:),'Color',plotColors(i,:),'LineWidth',1.2)
hold on
grid on
end

axis([-3000 3000 -2000 2000])
yline(0, 'LineWidth',1)
xline(0, 'LineWidth',1)
legend({'-0.113','0.113','-0.25','0.25','-0.5','0.5','-1','1','-5','5','-10','10','-50','50'},'Location','northeast','NumColumns',3);
title('Motion of Deputy for y0-dot variations')
xlabel('y[m]');
ylabel('x[m]');



figure(4)
X0 = [0 0 0 0 0 0]'; 
X0(1) = 50;
for i = 1:length(val_x_dot2)
X0(4) = val_x_dot2(i);
    for j = 1:length(t)
        X_t(:,j) = X(t(j),X0);
    end

plotColors = jet(length(val_x_dot2));
plot(X_t(2,:),X_t(1,:),'Color',plotColors(i,:),'LineWidth',1.2)
hold on
grid on
end

axis([-3000 3000 -2000 2000])
yline(0, 'LineWidth',1)
xline(0, 'LineWidth',1)
legend({'0','-0.5','0.5', '-1', '1', '-5', '5', '-10', '10', '-50', '50'},'Location','southeast','NumColumns',2);
title('Motion of Deputy for x0 displacement (50 m) and x0-dot variations')
xlabel('y[m]');
ylabel('x[m]');



figure(5)
X0 = [0 0 0 0 0 0]'; 
X0(1) = 50; 
for i = 1:length(val_y_dot2)
X0(5) = val_y_dot2(i);
    for j = 1:length(t)
        X_t(:,j) = X(t(j),X0);
    end

plotColors = jet(length(val_y_dot2));
plot(X_t(2,:),X_t(1,:),'Color',plotColors(i,:),'LineWidth',1.2)
hold on
grid on
end

axis([-3000 3000 -2000 2000])
yline(0, 'LineWidth',1)
xline(0, 'LineWidth',1)
legend({'-24','-12','-2.5','-1.3','-0.176','-0.117','0.058','1.05','2.2','12','24'},'Location','southeast','NumColumns',2);
title('Motion of Deputy for x0 displacement (50 m) and y0-dot variations')
xlabel('y[m]');
ylabel('x[m]');
