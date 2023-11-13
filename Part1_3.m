clc
clear all

disp(['Insert initial time (in minutes)',':'])
t_0 = input('Initial time:');  %[minutes]
t_0 = 60*t_0; %[s]
V_0 = [-0.1 -0.04 -0.02]; %[m/s]
x_0 = zeros(1,3); %[m]
h = 500*10^3; %[m]
mhu = 3.986004418*10^14; %[m^3 s^-2]
r_earth = 6371e3; %[m]
R_tgt = r_earth+h;
n = sqrt(mhu/(R_tgt)^3); % moto medio
tau = (pi/n);% Half orbital period [s]

[v,V,delta_v] = Vel_min(x_0, V_0, t_0, tau, n);




