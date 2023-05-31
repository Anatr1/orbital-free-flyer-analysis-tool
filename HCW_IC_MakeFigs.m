clear all
close all
clc

h = 500e3; % heigth
R = 6378.137e3; % Earth radius
mu = 3.986e14; % Earth Gravitation Parameter

n = sqrt(mu/(R+h)^3); % mean motion

A = [zeros(3), eye(3);[3*n^2 0 0 0 2*n 0; 0 0 0 -2*n 0 0; 0 0 -n^2 0 0 0]];
X0 = [0 0 0 0 0 0]'; % default initial conditions vector                      

phi = @(t) expm(A*t);
X = @(t,X0) phi(t)*X0;

t = linspace(0,200*60,1000);
X_t = zeros(length(X0),length(t)); 


val_x = [-5 5 -10 10 -50 50 -100 100 -500 500]; % array of different initial x values
val_x_dot = [-0.5 0.5 -1 1 -5 5 -10 10 -50 50]; % array of different initial x_dot values
val_x_dot2 = [0 -0.5 0.5 -1 1 -5 5 -10 10 -50 50]; % array of different initial x_dot values
val_y_dot1 = [-0.113 0.113 -0.25 0.25 -0.5 0.5 -1 1 -5 5 -10 10 -50 50]; % array of different initial y_dot values
val_y_dot2 = [-24 -12 -2.5 -1.3 -0.176 -0.117 0.058 1.05 2.2 12 24]; % array of different initial y_dot values



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

legend({'-5','5', '-10', '10', '-50', '50', '-100', '100', '-500', '500'},'Location','northeast','NumColumns',2);
axis([-3000 3000 -2000 2000])
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
legend({'0','-0.5','0.5', '-1', '1', '-5', '5', '-10', '10', '-50', '50'},'Location','southeast','NumColumns',2);
title('Motion of Deputy for x0 displacement (50 m) x0-dot variations')
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
legend({'-24','-12','-2.5','-1.3','-0.176','-0.117','0.058','1.05','2.2','12','24'},'Location','southeast','NumColumns',2);
title('Motion of Deputy for x0 displacement (50 m) y0-dot variations')
xlabel('y[m]');
ylabel('x[m]');
