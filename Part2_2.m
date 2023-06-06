% Vincenzo Trentacapilli, 
% Created 20233-05-11
% Prepared for ORDSS Class @ POLITO


close all
clc
format long

Part2


h = 500*10^3; %[m]
mhu = 3.986004418*10^14; %[m^3 s^-2]
r_earth = 6371e3; %[m]
R_tgt = r_earth+h;
n = sqrt(mhu/(R_tgt)^3); % moto medio
tau = (pi/n*2);% Half orbital period [s]
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% USER INPUT: Set Initial Configuration of the MBS

t0.theta0 = 0;                     % Orientation of the base
t0.PV.pos.r0.N = [0;0;0];    % Base (Link L0) CoM position
for i = 1 : 5
t0.qm(i) = 0;         % Displacement of the jointi -- thetai
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% USER INPUT: Set Initial Velocity of the MBS

t0.PV.Omega.L0.N.N = [0;0;0.01];      %angular velocity of base  -- non-zero along the axis of motion
t0.PV.dotN.pos.r0.N= zeros(3,1);    %absolute linear velocity of base COM
for i = 1 : 5
%     disp(['Insert joint', num2str(i),' speed [°/s]',':'])
%     t0.dot.qm(i) = input('speed:');     %Speed of the joint i
    t0.dot.qm(i) = 0;  
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Concatenate in a set the intial configuration
t0.DCM.L0.N = EulAxAngl2DCM([0;0;1], t0.theta0);  %DCM from N to L0 knowing theta0

q_BI_0 = DCM2quaternion(t0.DCM.L0.N);   % Initial Orientation of I wrt B of the base using orientation quaternion

q_IB_0 = quaternion2inversequat(q_BI_0);% Initial Orientation of B wrt I
Q_B_0 = [q_IB_0;t0.PV.pos.r0.N];        % Base initial configuration;  (7 ,1) matrix   
Q_0 = [Q_B_0; t0.qm(1); t0.qm(2); t0.qm(3); t0.qm(4); t0.qm(5)];                   % Base and manipulators initial configuration; (17 ,1) matrix


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% USER INPUT:  SET external or internal non-control wrenches
wFB = zeros(6,1);
wFm_1 = zeros(6,1);
wFm_2 = zeros(6,1);
wFm_3 = zeros(6,1);
wFm_4 = zeros(6,1);
wFm_5 = zeros(6,1);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% USER INPUT:  SET Control Actions
tau_control_B_Internal_Torque = zeros(3,1);
tau_control_B_External_Force = zeros(3,1);
tau_control_manipulator = zeros(1,5);
for i = 1 : 5
    tau_control_manipulator(i) = 0; 
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% USER INPUT: Set numerical propagation parameters

finaltime = 100;   %propagation time
propagation_timespan = [0,finaltime];

mytol = 1e-15; % relative and absolute tolerance of the propagator
% ODE options
ODEoptions = odeset('RelTol',mytol, 'AbsTol', mytol,'Stats','on', 'NormControl','on');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%  CODE  %%%%%

% Concatenate in a set the initial Velocity
uB_0 = [t0.PV.Omega.L0.N.N;t0.PV.dotN.pos.r0.N];  %Generalized velocity of the base 
u_0 = [uB_0;t0.dot.qm(1);t0.dot.qm(2); t0.dot.qm(3);t0.dot.qm(4);t0.dot.qm(5)];    %dot.qm1 angular rate at the joint 1  --  dot.qm2 angular rate at the joint 2

% Concatenate initial system state
initial_state = [Q_0;u_0];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Concatenate wrenches and control actions
NonControlWrenches = [wFB;wFm_1;wFm_2;wFm_3;wFm_4;wFm_5];
JointControlActions = [tau_control_B_Internal_Torque;tau_control_B_External_Force;tau_control_manipulator(1);tau_control_manipulator(2);tau_control_manipulator(3);tau_control_manipulator(4);tau_control_manipulator(5)];


% Execute Numerical Integration
[t_out, state_out] = ode45(@FFP6L5R_odefun_state2dotstate, propagation_timespan, initial_state, ODEoptions, JointControlActions, NonControlWrenches, robot); 

% EXPLANATION OF THE INTEGRATION OUTPUT
%
% t_out(i,1)        ith instant time at which solution is found
%
% CONFIGURATION AT TIME t_out(i,1), as follows:  
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
% VELOCITY AT TIME t_out(i,1), as follows:  
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



% Compute SYSTEM Momenta and SYSTEM CENTER OF MASS
MBB=robot.base_link.mass;
for i = 1 : 5
    MLL(i) = robot.links(i).mass;
end

%positions in plane of motion (for plotting)
com= zeros(size(state_out,1),3);
SCMt= zeros(size(state_out,1),2);
C0_t= zeros(size(state_out,1),2);
C1_t= zeros(size(state_out,1),2);
C2_t= zeros(size(state_out,1),2);
C3_t= zeros(size(state_out,1),2);
C4_t= zeros(size(state_out,1),2);
C5_t= zeros(size(state_out,1),2);

Oj1_t= zeros(size(state_out,1),2);
Oj2_t= zeros(size(state_out,1),2);
Oj3_t= zeros(size(state_out,1),2);
Oj4_t= zeros(size(state_out,1),2);
Oj5_t= zeros(size(state_out,1),2);

EE1_t= zeros(size(state_out,1),2);
EE2_t= zeros(size(state_out,1),2);
EE3_t= zeros(size(state_out,1),2);
EE4_t= zeros(size(state_out,1),2);
EE5_t= zeros(size(state_out,1),2);

MoMt_C0 = zeros(size(state_out,1),3);
Linear_Mom_CO=zeros(size(state_out,1),3);
MoMt_OI = zeros(size(state_out,1),3);
MoMt_sysC = zeros(size(state_out,1),3);

Axial_MoMm_j1 = zeros(size(state_out,1),1);
Axial_MoMm_j2 = zeros(size(state_out,1),1);
Axial_MoMm_j3 = zeros(size(state_out,1),1);
Axial_MoMm_j4 = zeros(size(state_out,1),1);
Axial_MoMm_j5 = zeros(size(state_out,1),1);

thetas=zeros(size(state_out,1),6);
dot_thetas=zeros(size(state_out,1),6);

for i=1:length(t_out)
    t = t_out(i);
    state = state_out(i,:)';
    C0_t(i,:)=state(5:6);
   % Recover Current Configuration
    q_IB = state(1:4);
    C_IB = Quat2DCM(q_IB);
    
    I_r_B = state(5:7);
    qm = state(8:12);

    [EulerAxis, theta0] =DCM2EulAxAngl_V2(C_IB');
    thetas(i,:)=[theta0,qm'];

    dot_thetas(i,:)=[state(15),state(19),state(20),state(21),state(22),state(23)];
    
    % Recover Current Velocity
    uB = state(13:18);
    um = state(19:23);
   
    % Compute kinematics
    [RJ,RL,rJ,rL,e,g] = Kinematics(C_IB,I_r_B,qm,robot);  %C_IB is a DCM -- RJ orientation of joint, rj position of the joint, e is the axis of the joint (0 0 1) because planar motion
    C1_t(i,:)=rL(1:2,1)';   %set for convenience
    Oj1_t(i,:)=rJ(1:2,1)';    %set for convenience
    C2_t(i,:)=rL(1:2,2)';   %set for convenience
    Oj2_t(i,:)=rJ(1:2,2)';    %set for convenience
    C3_t(i,:)=rL(1:2,3)';   %set for convenience
    Oj3_t(i,:)=rJ(1:2,3)';    %set for convenience
    C4_t(i,:)=rL(1:2,4)';   %set for convenience
    Oj4_t(i,:)=rJ(1:2,4)';    %set for convenience
    C5_t(i,:)=rL(1:2,5)';   %set for convenience
    Oj5_t(i,:)=rJ(1:2,5)';    %set for convenience
   
    posEEwrtC1_L1=(RL(:,:,1)*[l(1);0;0])';
    posEEwrtC2_L2=(RL(:,:,2)*[l(2);0;0])';
    posEEwrtC3_L3=(RL(:,:,3)*[l(3);0;0])';
    posEEwrtC4_L4=(RL(:,:,4)*[l(4);0;0])';
    posEEwrtC5_L5=(RL(:,:,5)*[l(5);0;0])';
 
    EE1_t(i,:)= C1_t(i,:) + posEEwrtC1_L1(1:2);
    EE2_t(i,:)= C2_t(i,:) + posEEwrtC2_L2(1:2);
    EE3_t(i,:)= C3_t(i,:) + posEEwrtC3_L3(1:2);
    EE4_t(i,:)= C4_t(i,:) + posEEwrtC4_L4(1:2);
    EE5_t(i,:)= C5_t(i,:) + posEEwrtC5_L5(1:2);

    

    % Compute differential kinematics
    [Bij,Bi0,P0,pm]=DiffKinematics(C_IB,I_r_B,rL,e,g,robot);   %pm velocity propagation vector (all of these are defined in the 3.6 or 3.5 slides)

    %Velocities
    [tB,tm]=Velocities(Bij,Bi0,P0,pm,uB,um,robot);   %Operational space velocities
    
    % Center of mass
    r_com = Center_of_Mass(I_r_B,rL,robot);
    com(i,:)=r_com';
    SCMt(i,:)=r_com(1:2)';
    % Save initial r_com
    if i==1
        r_com_0 = r_com;
    end
    dot_r_com = Center_of_Mass(tB(4:6),tm(4:6,:),robot);  %velocity of center of mass
      
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %MOMENTA COMPUTED FROM THE General Inertia Matrix
    
     %Inertias in inertial frames
    [I0,Im] = I_I(C_IB,RL,robot);   %Transfering the inertia matrix from CCS of L1 to CCS of Inertial Frame
    [M0_tilde,Mm_tilde] = MCB(I0,Im,Bij,Bi0,robot);    %MCB compute the generalized mass matrixes
    
    %Generalized Inertia matrix
    [H0, H0m, Hm] = GIM(M0_tilde,Mm_tilde,Bij,Bi0,P0,pm,robot);  %By using the definition
    
    Hall = [H0, H0m;H0m', Hm];

    MM_1 = (H0*uB+H0m*um)';
    
    MoMt_C0(i,:) = MM_1(1:3);   %Moment of momenta wrt C0 (will not conserve)
    MoMt_OI(i,:) = MoMt_C0(i,:)+((MBB+sum(MLL))*SkewSym(I_r_B)*dot_r_com)';   %Moment of momenta wrt OI (origin of Inertial CCS)(will conserve)

    OC = r_com;
    OC0 = I_r_B;

    CC0 = OC0-OC;
    MoMt_sysC(i,:) = MoMt_C0(i,:)+((MBB+sum(MLL))*SkewSym(CC0)*dot_r_com)';
   
%     MM_2=(H0m'*uB+Hm*um)';
%     
%     Axial_MoMm_j1(i,:)=MM_2(1);
%     Axial_MoMm_j2(i,:)=MM_2(2);
    
end
a = length(t_out);
x_5 = linspace(EE4_t(a,1),EE5_t(a,1));
m_L5 = (EE5_t(a,2) - EE4_t(a,2))/(EE5_t(a,1) - EE4_t(a,1)); 
q_L5 = EE4_t(a,2) - m_L5*EE4_t(a,1);
L5 = m_L5*x_5 + q_L5;

x_1 = linspace(Oj1_t(a,1),EE1_t(a,1));
m_L1 = (EE1_t(a,2) - Oj1_t(a,2))/(EE1_t(a,1) - Oj1_t(a,1)); 
q_L1 = Oj1_t(a,2) - m_L1*Oj1_t(a,1);
L1 = m_L1*x_1 + q_L1;

x_2 = linspace(EE1_t(a,1),EE2_t(a,1));
m_L2 = (EE2_t(a,2) - EE1_t(a,2))/(EE2_t(a,1) - EE1_t(a,1)); 
q_L2 = EE1_t(a,2) - m_L2*EE1_t(a,1);
L2 = m_L2*x_2 + q_L2;

x_3 = linspace(EE2_t(a,1),EE3_t(a,1));
m_L3 = (EE3_t(a,2) - EE2_t(a,2))/(EE3_t(a,1) - EE2_t(a,1)); 
q_L3 = EE2_t(a,2) - m_L3*EE2_t(a,1);
L3 = m_L3*x_3 + q_L3;

x_4 = linspace(EE3_t(a,1),EE4_t(a,1));
m_L4 = (EE4_t(a,2) - EE3_t(a,2))/(EE4_t(a,1) - EE3_t(a,1)); 
q_L4 = EE3_t(a,2) - m_L4*EE3_t(a,1);
L4 = m_L4*x_4 + q_L4;

x_v_C0 = [C0_t(a,1) - base_side/2, C0_t(a,1) + base_side/2, C0_t(a,1) + base_side/2,C0_t(a,1) - base_side/2, C0_t(a,1) - base_side/2];
y_v_C0 = [C0_t(a,2)- base_side/2, C0_t(a,2) - base_side/2, C0_t(a,2) + base_side/2, C0_t(a,2) + base_side/2, C0_t(a,2) - base_side/2];





