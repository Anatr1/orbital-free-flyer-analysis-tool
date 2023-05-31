clear all

h = 500*10^3; %[m]
mhu = 3.986004418*10^14; %[m^3 s^-2]
r_earth = 6371e3; %[m]
R_tgt = r_earth+h;
n = sqrt(mhu/(R_tgt)^3); % moto medio
tau = (pi/n);% Half orbital period [s]
V_0 = [ -0.1 -0.04 -0.02]; %[m/s]
x_0 = [0 0 0]; %[m]

% function [v,V,delta_v] = Vel_min(x_0, V_0, t_0, half_period, mean_motion)
omega = n;
% tau = half_period;
t = linspace(1,tau,1000);
t_0 = 10*60;

%First manuever
x(1) = (V_0(1)/omega)*sin(omega*t_0) - (2*V_0(2)/omega)*cos(omega*t_0) + (2*V_0(2)/omega);
x(2) = (4*V_0(2)/omega)*sin(omega*tau) + (2*V_0(1)/omega)*cos(omega*t_0) -  (2*V_0(1)/omega) - 3*V_0(2)*t_0;
x(3) = (V_0(3)/omega)*sin(omega*t_0);
x(4) = (V_0(1))*cos(omega*t_0) + (2*V_0(2))*sin(omega*t_0);
x(5) = (4*V_0(2))*cos(omega*t_0) - (2*V_0(1))*sin(omega*t_0) - (3*V_0(2)/omega);
x(6) = (V_0(3))*cos(omega*t_0);
    
state_t0 = [x(1) x(2) x(3) x(4) x(5) x(6)];

V_0 = state_t0(4:6);
x_0 = state_t0(1:3);

%Second maneuver
x_0_dot = zeros(1,3);
x_0_dot(2) = ((6*x_0(1)*(omega*tau - sin(omega*tau))) - x_0(2))*omega*sin(omega*tau) - 2*omega*x_0(1)*(4 - 3*cos(omega*tau))*(1 - cos(omega*tau))/((4*sin(omega*tau) - 3*omega*tau)*sin(omega*tau) + 4*(1 - cos(omega*tau))^2);
x_0_dot(1) = -(omega*x_0(1)*(4-3*cos(omega*tau))+2*x_0_dot(2)*(1-cos(omega*tau)))/(sin(omega*tau));
x_0_dot(3) = (-x_0(3))*(0.0011)*(cot(0.0011*tau));

for i = 1 : length(t)
x(1,i) = ((x_0_dot(1)*sin(omega*t(i)))/omega) - (3*x_0(1) + (2*x_0_dot(2))/omega)*cos(omega*t(i)) + (4*x_0(1)+ (2*x_0_dot(2))/omega);
x(2,i) = (6*x_0(1) + (4*x_0_dot(2))/omega)*sin(omega*t(i)) + ((2*x_0_dot(1)*cos(omega*t(i)))/omega) - (6*omega*x_0(1) + 3*x_0_dot(2))*t(i) + (x_0(2) - (2*x_0_dot(1))/omega);
x(3,i) = (x_0(3))*omega*cos(omega*t(i)) + ((x_0_dot(3)*sin(omega*t(i)))/omega);
x(4,i) = ((x_0_dot(1)*cos(omega*t(i)))) + (3*omega*x_0(1) + (2*x_0_dot(2)))*sin(omega*t(i));
x(5,i) = (6*omega*x_0(1) + (4*x_0_dot(2)))*cos(omega*t(i)) - ((2*x_0_dot(1)*sin(omega*t(i)))) - (6*omega*x_0(1) + 3*x_0_dot(2));
x(6,i) = -x_0(3)*omega*sin(omega*t(i)) + ((x_0_dot(3)*cos(omega*t(i))));
end
   
state = [x(1,i) x(2,i) x(3,i) x(4,i) x(5,i) x(6,i)];
v = state(4:6);
V = sqrt(state(4)^2 + state(5)^2 + state(6)^2);
delta_v = state(4:6) - V_0(1:3);
%end

