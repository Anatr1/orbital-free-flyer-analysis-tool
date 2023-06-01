clear all

h = 500*10^3; %[m]
mhu = 3.986004418*10^14; %[m^3 s^-2]
r_earth = 6371e3; %[m]
R_tgt = r_earth+h;
n = sqrt(mhu/(R_tgt)^3); % moto medio
tau = (pi/n);% Half orbital period [s]
v_0 = [ -0.1 -0.04 -0.02]; %[m/s]
x_0 = [0 0 0]; %[m]

%function [v_min,V_min,delta_V_min,t_vmin] = Vel_min_2(v_0, t_0, half_period, mean_motion)
omega = n;
%omega = mean_motion;
%tau = half_period;

 t_0 = 10*60;

% Manuever Zero
x(1) = (v_0(1)/omega)*sin(omega*t_0) - (2*v_0(2)/omega)*cos(omega*t_0) + (2*v_0(2)/omega);
x(2) = (4*v_0(2)/omega)*sin(omega*tau) + (2*v_0(1)/omega)*cos(omega*t_0) -  (2*v_0(1)/omega) - 3*v_0(2)*t_0;
x(3) = (v_0(3)/omega)*sin(omega*t_0);
x(4) = (v_0(1))*cos(omega*t_0) + (2*v_0(2))*sin(omega*t_0);
x(5) = (4*v_0(2))*cos(omega*t_0) - (2*v_0(1))*sin(omega*t_0) - (3*v_0(2)/omega);
x(6) = (v_0(3))*cos(omega*t_0);
    
state_t0 = [x(1) x(2) x(3) x(4) x(5) x(6)];

v_0 = state_t0(4:6);
V_0 = norm(v_0);
x_0 = state_t0(1:3);

a = 100; %intervalli di tempo
t1 = linspace(60,tau/1.5,a);

A = [zeros(3), eye(3);[3*n^2 0 0 0 2*n 0; 0 0 0 -2*n 0 0; 0 0 -n^2 0 0 0]];
X0 = state_t0'; % default initial conditions vector                      

phi = @(t) expm(A*t);
X = @(t,X0) phi(t)*X0;

X_t = zeros(length(X0),a);
V_fin = zeros(1,a);
delta_V_fin = zeros(1,a);
delta_v_fin = zeros(3,a);
indice = 0;
t_vmin = 0;
for j = 1:length(t1)
    t = linspace(1,t1(j),a);
    phi_t = phi(t1(j));
    if j==1
        for i = 1:length(t)
            X_t(:,i) = X(t(i),X0(:));
        end
    V_fin(j) = sqrt(X_t(4,100)^2 + X_t(5,100)^2 + X_t(6,100)^2); 
    delta_V_fin(j) = V_0 - V_fin(j);
    vmin_x = X_t(4,100);
    vmin_y = X_t(5,100);
    vmin_z = X_t(6,100);
    v_min = [vmin_x,vmin_y,vmin_z];
    delta_v_fin(:,j) = v_min - X0(4:6)';
    V_min = V_fin(j); 
    delta_V_min = delta_V_fin(j); 
    else
        for i = 1:length(t)
            X_t_1(:,i) = X(t(i),X0(:));
        end   
        V_fin(j) = sqrt(X_t_1(4,100)^2 + X_t_1(5,100)^2 + X_t_1(6,100)^2); 
        delta_V_fin(j) = V_0 - V_fin(j);
        %somma con il primo deltaV
        %I = somma deltaVs
        if delta_V_fin(j) < delta_V_fin(j-1)
            vmin_x = X_t_1(4,100);
            vmin_y = X_t_1(5,100);
            vmin_z = X_t_1(6,100);
            v_min = [vmin_x,vmin_y,vmin_z];
            delta_v_fin(:,j) = v_min - X0(4:6)';
            V_min = V_fin(j);
            delta_V_min = delta_V_fin(j);
            delta_v_min = delta_v_fin(j);
            t_vmin = t1(j);
            indice = j;
        end
    end
end