




function [Delta_v, t_DV_min ] = TwoImp_min(PosVelChief, mean_motion, Orbital_period)
n = mean_motion;
T = Orbital_period;
tau = T/2;
n_imp = 2;
X0 = PosVelChief;


A = [zeros(3),eye(3);[3*(n^2) 0 0 0 2*n 0; 0 0 0 -2*n 0 0; 0 0 -n^2 0 0 0]];
phi = @(t)    expm(A*t);
X   = @(t,X0) phi(t)*X0;
XF = [0 0 0 0 0 0]';

t_DV_min=0;% tempo di impulso inizializzazione
Delta_V=1e15; %valore di inzializzazione molto grande per un delta V di inizializzazione

for i=1:tau

    Pos_chief=XF(1:3);
    Pos_in=X0(1:3);
    Phi_t=phi(i);
    Vi=Phi_t(1:3,4:6)\(Pos_chief-Phi_t(1:3,1:3)*Pos_in); %velocità iniziali

%First impulse components wrt chief's position
    Delta_v(1,i)= Vi(1)-X0(4);
    Delta_v(2,i)= Vi(2)-X0(5);
    Delta_v(3,i)= Vi(3)-X0(6);
%Final velocity and position computation after first impulse
Xi=[X0(1:3)',Vi']'; 
Xf=X(i,Xi); 
Delta_V_2=-Xf(4:6); 
Total_Impulse=sqrt(Delta_v(1,i)^2+Delta_v(2,i)^2+Delta_v(3,i)^2)+sqrt(Delta_V_2(1)^2+Delta_V_2(2)^2+Delta_V_2(3)^2);
    if Total_Impulse<Delta_V 
        Delta_V=Total_Impulse;
        t_DV_min=i;
    end
   
end




   