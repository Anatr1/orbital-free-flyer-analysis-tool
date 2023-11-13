


function [Delta_V, Delta_v_comp, t_DV_min ] = TwoImp_min(PosVelDeputy,PosVelChief, mean_motion, Orbital_period)
% ORBITAL ROBOTICS AND DISPUTED SPACE SYSTEMS
% VINCENZO TRENTACAPILLI, GAETANA GAIA SPANO', LORENZO PORPIGLIA, FEDERICO MUSTICH

%  The function TwoImp_min, computes the minimun total V two-impulse
% maneuvers, knowing the initial conditions of position and velocity of 
% Deputy w.r.t. target. To do this, the function considers a maximum final 
% time equal to half orbit period.   

% Initial conditions of position and velocity are given as components of 
% two vectors (total of six parameters).
% After Clohessy-Wiltshire equations are solved to compute the minimum
% Delta_V and the corresponding time for the Deputy to reach the target.

%
% INPUTS
% PosVelDeputy = [1,6 matrix of real numbers describing the components of position and velocity vector of the deputy wrt chief's position]
% PosVelChief = [1,6 matrix of real numbers describing the components of position and velocity vector of the chief]
% mean_motion = scalar number of the mean motion [s^-1]
% Orbital_period = Orbital periond [s]
% 
% OUTPUTS
% Delta_V = magnitude of the total impulse equivalent to a two impulse maneuver
% Delta_v_comp = [1,3 matrix of real numers representive the three components of the total impulse equivalent to a two impulse maneuver]
% t_DV_min = minimum time to execute the maneuver of rendezvous [s]


n = mean_motion;  % [s^-1]      
T = Orbital_period; % [s]    
tau = T/2;           % [s]              
n_imp = 2;              % Impulses number          
X0 = PosVelDeputy';     % Deputy positions and velocities [m] [m/s]


A = [zeros(3),eye(3);[3*(n^2) 0 0 0 2*n 0; 0 0 0 -2*n 0 0; 0 0 -n^2 0 0 0]];
phi = @(t)    expm(A*t);
X   = @(t,X0) phi(t)*X0;
XF = PosVelChief';


t_DV_min = 0;           
Delta_V = 1e15;         

for i = 1 : tau

    Pos_chief = XF(1:3);
    Vel_chief = XF(4:6);
    Pos_in = X0(1:3);
    Phi_t = phi(i);
    Vi = (Phi_t(1:3,4:6))\(Pos_chief-Phi_t(1:3,1:3)*Pos_in); %Initial velocities

%First impulse components wrt chief's position
    Delta_v(1,i) = Vi(1) - X0(4);
    Delta_v(2,i) = Vi(2) - X0(5);
    Delta_v(3,i) = Vi(3) - X0(6);
%Final velocity and position computation after first impulse
Xi = [X0(1:3)',Vi']'; 
Xf = X(i,Xi); 
Delta_V_2 = -Xf(4:6); 
Total_Impulse = sqrt(Delta_v(1,i)^2 + Delta_v(2,i)^2 + Delta_v(3,i)^2) + sqrt(Delta_V_2(1)^2 + Delta_V_2(2)^2 + Delta_V_2(3)^2);
    if Total_Impulse < Delta_V 
        Delta_V = Total_Impulse;
        t_DV_min = i;
        Delta_v_comp = Delta_v(:,i) + Delta_V_2(:);
    end
   
end
end




