function dot_state = odefun_state_2_dotstate(t, state,  ControlActions, NonControlWrenches, robot)
h = 500*10^3; %[m]
mhu = 3.986004418*10^14; %[m^3 s^-2]
r_earth = 6371e3; %[m]
R_tgt = r_earth+h;
n = sqrt(mhu/(R_tgt)^3); % moto medio
% CONFIGURATION AT TIME t, provided by the numerical integrator 
% state_out(i,1)   q_IB(1) at time t_out(i,1) 
% state_out(i,2)   q_IB(2) at time t_out(i,1) 
% state_out(i,3)   q_IB(3) at time t_out(i,1) 
% state_out(i,4)   q_IB(4) at time t_out(i,1)   

% state_out(i,5)   I_r_B(1) at time t_out(i,1)   
% state_out(i,6)   I_r_B(2) at time t_out(i,1)   
% state_out(i,7)   I_r_B(3) at time t_out(i,1) 

% state_out(i,8)   qm1 at time t_out(i,1)
% state_out(i,9)   qm2 at time t_out(i,1)
% state_out(i,10)   qm3 at time t_out(i,1)
% state_out(i,11)   qm4 at time t_out(i,1)
% state_out(i,12)   qm5 at time t_out(i,1)
%
% VELOCITY AT TIME t_out(i,1), provided by the numerical integrator     
% state_out(i,13)    I_Omega_BI(1) at time t_out(i,1)  
% state_out(i,14)   I_Omega_BI(2) at time t_out(i,1) 
% state_out(i,15)   I_Omega_BI(3) at time t_out(i,1) 
% state_out(i,16)   I_dot_r_B(1) at time t_out(i,1) 
% state_out(i,17)   I_dot_r_B(2) at time t_out(i,1) 
% state_out(i,18)   I_dot_r_B(3) at time t_out(i,1) 
% state_out(i,19)   dot_qm1 at time t_out(i,1) 
% state_out(i,20)   dot_qm2 at time t_out(i,1) 
% state_out(i,21)   dot_qm3 at time t_out(i,1) 
% state_out(i,22)   dot_qm4 at time t_out(i,1) 
% state_out(i,23)   dot_qm5 at time t_out(i,1) 


% Recover Current Configuration
q_IB = state(1:4);    %/norm(state(1:4));
C_IB = quat_DCM(q_IB);
I_r_B = state(5:7);
qm = state(8:12);

% Recover Current Velocity
uB = state(13:18);
um = state(19:23);

% Compute kinematics
[RJ,RL,rJ,I_r_L,e,g] = Kinematics(C_IB,I_r_B,qm,robot);

%Inertias in inertial frames
[IB,Im] = I_I(C_IB,RL,robot);

% Compute differential kinematics
[Bij,Bi0,P0,pm] = DiffKinematics(C_IB,I_r_B,I_r_L,e,g,robot);

%Operational Velocities
[tB,tm] = Velocities(Bij,Bi0,P0,pm,uB,um,robot);

%Forward Dynamics
tau_control_B = ControlActions(1:6);
tau_control_joint = ControlActions(7:11);

% NonControlWrenches(:,1) -> WB
% NonControlWrenches(:,2) -> Wm_1
% NonControlWrenches(:,3) -> Wm_2
% NonControlWrenches(:,4) -> WB_3
% NonControlWrenches(:,5) -> Wm_4
% NonControlWrenches(:,6) -> Wm_5


I = zeros(3,3,6);
I(:,:,1) = C_IB *robot.base_link.inertia* C_IB';
for i = 1 : 5
    I(:,:,i+1) = RL(:,:,i)'*robot.links(i).inertia*RL(:,:,i);
end

h1 = [-cos(n*t); -sin(n*t); 0];

WB = (3*mhu/R_tgt^3)*[cross(h1, I(:,:,1)*h1); 0;0;0];
Wm_1 = (3*mhu/R_tgt^3)*[cross(h1, I(:,:,2)*h1); 0;0;0];
Wm_2 = (3*mhu/R_tgt^3)*[cross(h1, I(:,:,3)*h1); 0;0;0];
Wm_3 = (3*mhu/R_tgt^3)*[cross(h1, I(:,:,4)*h1); 0;0;0];
Wm_4 = (3*mhu/R_tgt^3)*[cross(h1, I(:,:,5)*h1); 0;0;0];
Wm_5 = (3*mhu/R_tgt^3)*[cross(h1, I(:,:,6)*h1); 0;0;0];
Wm_f = [Wm_1,Wm_2,Wm_3,Wm_4,Wm_5];

[uBdot,umdot] = FD(tau_control_B,tau_control_joint,WB,Wm_f,tB,tm,P0,pm,IB,Im,Bij,Bi0,uB,um,robot);

% Get quaternion velocity
I_Omega_BI = uB(1:3);
q1 = q_IB(1);
q2 = q_IB(2);
q3 = q_IB(3);
q4 = q_IB(4);
dot_q_IB = 0.5*[q4,-q3,q2,q1; q3, q4,-q1,q2; -q2, q1,q4,q3; -q1, -q2,-q3,q4]*[-I_Omega_BI(1);-I_Omega_BI(2);-I_Omega_BI(3);0];
% The code finally outputs a matrix of 23 columns, which also corresponds
% to the columns of the state matrix. The propagation is described using a
% formula which we saw during the lessons about quaternions a and
% Differential Kinematics equations.
dot_state=[dot_q_IB;uB(4:6);um;uBdot;umdot];
end
