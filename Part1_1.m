clc
clear all
colordef white
%% Example 6-14
%Initial conditions of Velocity and Position
V_0 = [ -0.1 -0.04 -0.02]; %[m/s]
x_0 = [0 0 0]; %[m]
Initial_conditions = [0 0 0 -0.1 -0.04 -0.02];

t_initial = 0;
t_final_5 = 5*60;   %[sec]
t_final_20 = 20*60; 
delta_t_5 = [t_initial t_final_5];
delta_t_20 = [t_initial t_final_20];

h = 500*10^3; %[m]
mhu = 3.986004418*10^14; %[m^3 s^-2]
r_earth = 6371e3; %[m]
R_tgt = r_earth+h;
n = sqrt(mhu/(R_tgt)^3); % moto medio
tau = (pi/n);% Half orbital period [s]


%Angular Velocity
omega = sqrt(mhu/R_tgt^3);

%5 minutes   
x(1) = (V_0(1)/omega)*sin(omega*t_final_5) - (2*V_0(2)/omega)*cos(omega*t_final_5) + (2*V_0(2)/omega);
x(2) = (4*V_0(2)/omega)*sin(omega*t_final_5) + (2*V_0(1)/omega)*cos(omega*t_final_5) -  (2*V_0(1)/omega) - 3*V_0(2)*t_final_5;
x(3) = (V_0(3)/omega)*sin(omega*t_final_5);
x(4) = (V_0(1))*cos(omega*t_final_5) + (2*V_0(2))*sin(omega*t_final_5);
x(5) = (4*V_0(2))*cos(omega*t_final_5) - (2*V_0(1))*sin(omega*t_final_5) - (3*V_0(2)/omega);
x(6) = (V_0(3))*cos(omega*t_final_5);
    
state_5 = [x(1) x(2) x(3) x(4) x(5) x(6)];

%20 minutes
x(1) = (V_0(1)/omega)*sin(omega*t_final_20) - (2*V_0(2)/omega)*cos(omega*t_final_20) + (2*V_0(2)/omega);
x(2) = (4*V_0(2)/omega)*sin(omega*t_final_20) + (2*V_0(1)/omega)*cos(omega*t_final_20) -  (2*V_0(1)/omega) - 3*V_0(2)*t_final_20;
x(3) = (V_0(3)/omega)*sin(omega*t_final_20);
x(4) = (V_0(1))*cos(omega*t_final_20) + (2*V_0(2))*sin(omega*t_final_20);
x(5) = (4*V_0(2))*cos(omega*t_final_20) - (2*V_0(1))*sin(omega*t_final_20) - (3*V_0(2)/omega);
x(6) = (V_0(3))*cos(omega*t_final_20);
    
state_20 = [x(1) x(2) x(3) x(4) x(5) x(6)];

%% Example 6-15
t = linspace(0,300,300);
%10  minutes
t_final_10 = 10*60; 
delta_t_10 = [t_initial t_final_10];

x(1) = (V_0(1)/omega)*sin(omega*t_final_10) - (2*V_0(2)/omega)*cos(omega*t_final_10) + (2*V_0(2)/omega);
x(2) = (4*V_0(2)/omega)*sin(omega*t_final_10) + (2*V_0(1)/omega)*cos(omega*t_final_10) -  (2*V_0(1)/omega) - 3*V_0(2)*t_final_10;
x(3) = (V_0(3)/omega)*sin(omega*t_final_10);
x(4) = (V_0(1))*cos(omega*t_final_10) + (2*V_0(2))*sin(omega*t_final_10);
x(5) = (4*V_0(2))*cos(omega*t_final_10) - (2*V_0(1))*sin(omega*t_final_10) - (3*V_0(2)/omega);
x(6) = (V_0(3))*cos(omega*t_final_10);
    
state_10 = [x(1) x(2) x(3) x(4) x(5) x(6)];

V_0 = state_10(4:6);
x_0 = state_10(1:3);

t_final_5_n = 47*60;
t_final_15_n = 15*60;

% 5 minutes rendezvous
x_0_dot = zeros(1,3);
x_0_dot(2) = ((6*x_0(1)*(omega*t_final_5_n - sin(omega*t_final_5_n)) - x_0(2))*omega*sin(omega*t_final_5_n) - 2*omega*x_0(1)*(4 - 3*cos(omega*t_final_5_n))*(1 - cos(omega*t_final_5_n)))/((4*sin(omega*t_final_5_n) - 3*omega*t_final_5_n)*sin(omega*t_final_5_n) + 4*(1 - cos(omega*t_final_5_n))^2);
x_0_dot(1) = -(omega*x_0(1)*(4-3*cos(omega*t_final_5_n))+2*x_0_dot(2)*(1-cos(omega*t_final_5_n)))/(sin(omega*t_final_5_n));
x_0_dot(3) = (-x_0(3))*omega*cot(omega*t_final_5_n);

for i = 1 : length(t)
x(1,i) = ((x_0_dot(1)*sin(omega*t(i)))/omega) - (3*x_0(1) + (2*x_0_dot(2))/omega)*cos(omega*t(i)) + (4*x_0(1)+ (2*x_0_dot(2))/omega);
x(2,i) = (6*x_0(1) + (4*x_0_dot(2))/omega)*sin(omega*t(i)) + ((2*x_0_dot(1)*cos(omega*t(i)))/omega) - (6*omega*x_0(1) + 3*x_0_dot(2))*t(i) + (x_0(2) - (2*x_0_dot(1))/omega);
x(3,i) = (x_0(3))*omega*cos(omega*t(i)) + ((x_0_dot(3)*sin(omega*t(i)))/omega);
x(4,i) = ((x_0_dot(1)*cos(omega*t(i)))) + (3*omega*x_0(1) + (2*x_0_dot(2)))*sin(omega*t(i));
x(5,i) = (6*omega*x_0(1) + (4*x_0_dot(2)))*cos(omega*t(i)) - ((2*x_0_dot(1)*sin(omega*t(i)))) - (6*omega*x_0(1) + 3*x_0_dot(2));
x(6,i) = -x_0(3)*omega*sin(omega*t(i)) + ((x_0_dot(3)*cos(omega*t(i))));
end
state_5_n = [x(1,300) x(2,300) x(3,300) x(4,300) x(5,300) x(6,300)];
V_final_5_n = sqrt(state_5_n(4)^2 + state_5_n(5)^2 + state_5_n(6)^2);
delta_v_5 = state_5_n(4:6) - state_10(4:6);

% 15 minutes rendezvous
% x_0_dot = zeros(1,3);
% x_0_dot(2) = ((6*x_0(1)*(omega*t_final_15_n - sin(omega*t_final_15_n)) - x_0(2))*omega*sin(omega*t_final_15_n) - 2*omega*x_0(1)*(4 - 3*cos(omega*t_final_15_n))*(1 - cos(omega*t_final_15_n)))/((4*sin(omega*t_final_15_n) - 3*omega*t_final_15_n)*sin(omega*t_final_15_n) + 4*(1 - cos(omega*t_final_15_n))^2);
% x_0_dot(1) = -(omega*x_0(1)*(4-3*cos(omega*t_final_15_n))+2*x_0_dot(2)*(1-cos(omega*t_final_15_n)))/(sin(omega*t_final_15_n));
% x_0_dot(3) = (-x_0(3))*omega*cot(omega*t_final_15_n);
% 
% x(1) = ((x_0_dot(1)*sin(omega*t_final_15_n))/omega) - (3*x_0(1) + (2*x_0_dot(2))/omega)*cos(omega*t_final_15_n) + (4*x_0(1)+ (2*x_0_dot(2))/omega);
% x(2) = (6*x_0(1) + (4*x_0_dot(2))/omega)*sin(omega*t_final_15_n) + ((2*x_0_dot(1)*cos(omega*t_final_15_n))/omega) - (6*omega*x_0(1) + 3*x_0_dot(2))*t_final_15_n + (x_0(2) - (2*x_0_dot(1))/omega);
% x(3) = (x_0(3))*omega*cos(omega*t_final_15_n) + ((x_0_dot(3)*sin(omega*t_final_15_n))/omega);
% x(4) = ((x_0_dot(1)*cos(omega*t_final_15_n))) + (3*omega*x_0(1) + (2*x_0_dot(2)))*sin(omega*t_final_15_n);
% x(5) = (6*omega*x_0(1) + (4*x_0_dot(2)))*cos(omega*t_final_15_n) - ((2*x_0_dot(1)*sin(omega*t_final_15_n))) - (6*omega*x_0(1) + 3*x_0_dot(2));
% x(6) = -x_0(3)*omega*sin(omega*t_final_15_n) + ((x_0_dot(3)*cos(omega*t_final_15_n)));
% 
% state_15_n = [x(1) x(2) x(3) x(4) x(5) x(6)];
% V_final_15_n = sqrt(state_15_n(4)^2 + state_15_n(5)^2 + state_15_n(6)^2);
% delta_v_15 = state_15_n(4:6) - state_10(4:6);


% X_t = zeros(3,length(t));
% for i = 1 : length(t)
%     X_t(1,i) = x(1,i);
%     X_t(2,i) = x(2,i);
%     X_t(3,i) = x(3,i);
% end
% 
% figure
% plot3(X_t(1,:),X_t(2,:),X_t(3,:),'b--', 'LineWidth',0.5);



    