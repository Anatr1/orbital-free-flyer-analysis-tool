% FINAL PROJECT --- GROUP 18
%
% Federico Mustich, Lorenzo Porpiglia, Gaetana Gaia Spanò, Vincenzo Trentacapilli
%
% Starting from the data stored in "robot" the script computes the motion of 
% the moving base robotic manipulator following the process of the recursive 
% analysis, by developing the equations of motion and numerically integrating them. 
% The evolution of the motion, while no torques are acting on the revolute joints, 
% is studied and the gravitational effect is included. the freeflyer is at rest 
% at the beginning of the simulation.
%

close all
clc
format long

FFP6R5L_Robot_Description

    
h = 500*10^3;               % Altitude [m]
mhu = 3.986004418*10^14;    % Gravitational standard costant [m^3 s^-2]
r_earth = 6371e3;           % Earth radius [m]
R_tgt = r_earth+h;          % Target radius [m]
n = sqrt(mhu/(R_tgt)^3);    % Mean motion [rad/s]
tau = (pi/n*2);             % Half orbital period [s]
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% USER INPUT: Set Initial Configuration of the MBS

t0.theta0 = 0;                     % Orientation of the base
t0.PV.pos.r0.N = [0;0;0];          % Base (Link L0) CoM position
for i = 1 : 5
    t0.qm(i) = 0;                  % Displacement of the jointi -- theta-i [rad]
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% USER INPUT: Set Initial Velocity of the MBS

t0.PV.Omega.L0.N.N = [0;0;0.01];      % Angular velocity of base  -- non-zero along the axis of motion
t0.PV.dotN.pos.r0.N = [0;0;0];      % Absolute linear velocity of base COM
for i = 1 : 5 
    t0.dot.qm(i) = 0;                 %Speed of the joint i
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Concatenate in a set the intial configuration
t0.DCM.L0.N = EulAxAngl2DCM([0;0;1], t0.theta0);                    % DCM from N to L0 knowing theta0

q_BI_0 = DCM2quaternion(t0.DCM.L0.N);                               % Initial Orientation of I wrt B of the base using orientation quaternion

q_IB_0 = quaternion2inversequat(q_BI_0);                            % Initial Orientation of B wrt I
Q_B_0 = [q_IB_0;t0.PV.pos.r0.N];                                    % Base initial configuration;  (7 ,1) matrix   
Q_0 = [Q_B_0; t0.qm(1); t0.qm(2); t0.qm(3); t0.qm(4); t0.qm(5)];    % Base and manipulators initial configuration; (17 ,1) matrix


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% USER INPUT:  SET external or internal non-control wrenches --- Gravity Gradient implementation
wFB = (3*mhu/R_tgt^3)* [(robot.base_link.inertia(3,3) - robot.base_link.inertia(2,2))*t0.DCM.L0.N(2,1)*t0.DCM.L0.N(3,1);
                                                 (robot.base_link.inertia(1,1) - robot.base_link.inertia(3,3))*t0.DCM.L0.N(1,1)*t0.DCM.L0.N(3,1);
                                                 (robot.base_link.inertia(2,2) - robot.base_link.inertia(1,1))*t0.DCM.L0.N(2,1)*t0.DCM.L0.N(1,1);0;0;0];
wFm_1 = (3*mhu/R_tgt^3)* [(robot.links(1).inertia(3,3) - robot.links(1).inertia(2,2))*t0.DCM.L0.N(2,1)*t0.DCM.L0.N(3,1);
                                                 (robot.links(1).inertia(1,1) - robot.links(1).inertia(3,3))*t0.DCM.L0.N(1,1)*t0.DCM.L0.N(3,1);
                                                 (robot.links(1).inertia(2,2) - robot.links(1).inertia(1,1))*t0.DCM.L0.N(2,1)*t0.DCM.L0.N(1,1);0;0;0];
wFm_2 = (3*mhu/R_tgt^3)* [(robot.links(2).inertia(3,3) - robot.links(2).inertia(2,2))*t0.DCM.L0.N(2,1)*t0.DCM.L0.N(3,1);
                                                 (robot.links(2).inertia(1,1) - robot.links(2).inertia(3,3))*t0.DCM.L0.N(1,1)*t0.DCM.L0.N(3,1);
                                                 (robot.links(2).inertia(2,2) - robot.links(2).inertia(1,1))*t0.DCM.L0.N(2,1)*t0.DCM.L0.N(1,1);0;0;0];
wFm_3 = (3*mhu/R_tgt^3)* [(robot.links(3).inertia(3,3) - robot.base_link.inertia(2,2))*t0.DCM.L0.N(2,1)*t0.DCM.L0.N(3,1);
                                                 (robot.links(3).inertia(1,1) - robot.links(3).inertia(3,3))*t0.DCM.L0.N(1,1)*t0.DCM.L0.N(3,1);
                                                 (robot.links(3).inertia(2,2) - robot.links(3).inertia(1,1))*t0.DCM.L0.N(2,1)*t0.DCM.L0.N(1,1);0;0;0];
wFm_4 = (3*mhu/R_tgt^3)* [(robot.links(4).inertia(3,3) - robot.links(4).inertia(2,2))*t0.DCM.L0.N(2,1)*t0.DCM.L0.N(3,1);
                                                 (robot.links(4).inertia(1,1) - robot.links(4).inertia(3,3))*t0.DCM.L0.N(1,1)*t0.DCM.L0.N(3,1);
                                                 (robot.links(4).inertia(2,2) - robot.links(4).inertia(1,1))*t0.DCM.L0.N(2,1)*t0.DCM.L0.N(1,1);0;0;0];
wFm_5 = (3*mhu/R_tgt^3)* [(robot.links(5).inertia(3,3) - robot.links(5).inertia(2,2))*t0.DCM.L0.N(2,1)*t0.DCM.L0.N(3,1);
                                                 (robot.links(5).inertia(1,1) - robot.links(5).inertia(3,3))*t0.DCM.L0.N(1,1)*t0.DCM.L0.N(3,1);
                                                 (robot.links(5).inertia(2,2) - robot.links(5).inertia(1,1))*t0.DCM.L0.N(2,1)*t0.DCM.L0.N(1,1);0;0;0];
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
uB_0 = [t0.PV.Omega.L0.N.N;t0.PV.dotN.pos.r0.N];                                     %Generalized velocity of the base 
u_0 = [uB_0;t0.dot.qm(1);t0.dot.qm(2); t0.dot.qm(3);t0.dot.qm(4);t0.dot.qm(5)];      %dot.qm1 angular rate at the joint 1  --  dot.qm2 angular rate at the joint 2 -- ecc.

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

% Positions in plane of motion (for plotting)
com = zeros(size(state_out,1),3);
SCMt = zeros(size(state_out,1),2);
C0_t = zeros(size(state_out,1),2);
C1_t = zeros(size(state_out,1),2);
C2_t = zeros(size(state_out,1),2);
C3_t = zeros(size(state_out,1),2);
C4_t = zeros(size(state_out,1),2);
C5_t = zeros(size(state_out,1),2);

Oj1_t = zeros(size(state_out,1),2);
Oj2_t = zeros(size(state_out,1),2);
Oj3_t = zeros(size(state_out,1),2);
Oj4_t = zeros(size(state_out,1),2);
Oj5_t = zeros(size(state_out,1),2);

EE1_t = zeros(size(state_out,1),2);
EE2_t = zeros(size(state_out,1),2);
EE3_t = zeros(size(state_out,1),2);
EE4_t = zeros(size(state_out,1),2);
EE5_t = zeros(size(state_out,1),2);

MoMt_C0 = zeros(size(state_out,1),3);
Linear_Mom_CO = zeros(size(state_out,1),3);
MoMt_OI = zeros(size(state_out,1),3);
MoMt_sysC = zeros(size(state_out,1),3);

Axial_MoMm_j1 = zeros(size(state_out,1),1);
Axial_MoMm_j2 = zeros(size(state_out,1),1);
Axial_MoMm_j3 = zeros(size(state_out,1),1);
Axial_MoMm_j4 = zeros(size(state_out,1),1);
Axial_MoMm_j5 = zeros(size(state_out,1),1);

thetas = zeros(size(state_out,1),6);
dot_thetas = zeros(size(state_out,1),6);

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
    C1_t(i,:) = rL(1:2,1)';   %set for convenience
    Oj1_t(i,:) = rJ(1:2,1)';    %set for convenience
    C2_t(i,:) = rL(1:2,2)';   %set for convenience
    Oj2_t(i,:) = rJ(1:2,2)';    %set for convenience
    C3_t(i,:) = rL(1:2,3)';   %set for convenience
    Oj3_t(i,:) = rJ(1:2,3)';    %set for convenience
    C4_t(i,:) = rL(1:2,4)';   %set for convenience
    Oj4_t(i,:) = rJ(1:2,4)';    %set for convenience
    C5_t(i,:) = rL(1:2,5)';   %set for convenience
    Oj5_t(i,:) = rJ(1:2,5)';    %set for convenience
   
    posEEwrtC1_L1 = (RL(:,:,1)*[l(1);0;0])';
    posEEwrtC2_L2 = (RL(:,:,2)*[l(2);0;0])';
    posEEwrtC3_L3 = (RL(:,:,3)*[l(3);0;0])';
    posEEwrtC4_L4 = (RL(:,:,4)*[l(4);0;0])';
    posEEwrtC5_L5 = (RL(:,:,5)*[l(5);0;0])';
 
    EE1_t(i,:) = C1_t(i,:) + posEEwrtC1_L1(1:2);
    EE2_t(i,:) = C2_t(i,:) + posEEwrtC2_L2(1:2);
    EE3_t(i,:) = C3_t(i,:) + posEEwrtC3_L3(1:2);
    EE4_t(i,:) = C4_t(i,:) + posEEwrtC4_L4(1:2);
    EE5_t(i,:) = C5_t(i,:) + posEEwrtC5_L5(1:2);

    

    % Compute differential kinematics
    [Bij,Bi0,P0,pm] = DiffKinematics(C_IB,I_r_B,rL,e,g,robot);   

    %Velocities
    [tB,tm] = Velocities(Bij,Bi0,P0,pm,uB,um,robot);   % Operational space velocities
    
    % Center of mass
    r_com = Center_of_Mass(I_r_B,rL,robot);
    com(i,:) = r_com';
    SCMt(i,:) = r_com(1:2)';
    % Save initial r_com
    if i == 1
        r_com_0 = r_com;
    end
    dot_r_com = Center_of_Mass(tB(4:6),tm(4:6,:),robot);  %velocity of center of mass
      
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %MOMENTA COMPUTED FROM THE General Inertia Matrix
    
     %Inertias in inertial frames
    [I0,Im] = I_I(C_IB,RL,robot);                      %Transfering the inertia matrix from CCS of L1 to CCS of Inertial Frame
    [M0_tilde,Mm_tilde] = MCB(I0,Im,Bij,Bi0,robot);    %MCB compute the generalized mass matrixes
    
    %Generalized Inertia matrix
    [H0, H0m, Hm] = GIM(M0_tilde,Mm_tilde,Bij,Bi0,P0,pm,robot);  %By using the definition
    
    Hall = [H0, H0m;H0m', Hm];

    MM_1 = (H0*uB+H0m*um)';
    
    MoMt_C0(i,:) = MM_1(1:3);                                                 %Moment of momenta wrt C0 (will not conserve)
    MoMt_OI(i,:) = MoMt_C0(i,:)+((MBB+sum(MLL))*SkewSym(I_r_B)*dot_r_com)';   %Moment of momenta wrt OI (origin of Inertial CCS)(will conserve)

    OC = r_com;
    OC0 = I_r_B;

    CC0 = OC0-OC;
    MoMt_sysC(i,:) = MoMt_C0(i,:)+((MBB+sum(MLL))*SkewSym(CC0)*dot_r_com)';
   
    MM_2 = (H0m'*uB+Hm*um)';
    
    Axial_MoMm_j1(i,:) = MM_2(1);
    Axial_MoMm_j2(i,:) = MM_2(2);
    Axial_MoMm_j3(i,:) = MM_2(3);
    Axial_MoMm_j4(i,:) = MM_2(4);
    Axial_MoMm_j5(i,:) = MM_2(5);
    
end

for i = 1 : length(t_out)
a = length(t_out);

% Links motion -- for plotting
x_5(i,:) = linspace(EE4_t(i,1),EE5_t(i,1));
m_L5(i) = (EE5_t(i,2) - EE4_t(i,2))/(EE5_t(i,1) - EE4_t(i,1)); 
q_L5(i) = EE4_t(i,2) - m_L5(i)*EE4_t(i,1);
L5(i,:) = m_L5(i)*x_5(i,:) + q_L5(i);
x_5_top(i,:) = x_5(i,:) - (man_width(5)/2)*cos(-pi/4);
x_5_bot(i,:) = x_5(i,:) + (man_width(5)/2)*cos(-pi/4);
L5_x_rect_top(i,:) = L5(i,:) + (man_width(5)/2)*cos(-pi/4);
L5_x_rect_bot(i,:) = L5(i,:) - (man_width(5)/2)*cos(-pi/4);
x_5_1(i,:) = linspace(EE4_t(i,1) - (man_width(5)/2)*cos(pi/4),EE4_t(i,1) + (man_width(5)/2)*cos(pi/4));
x_5_100(i,:) = linspace(EE5_t(i,1) - (man_width(5)/2)*cos(pi/4),EE5_t(i,1) + (man_width(5)/2)*cos(pi/4));
m_L5_1(i) = (L5_x_rect_top(i,1) - L5_x_rect_bot(i,1))/(x_5_1(i,1) - x_5_1(i,100)); 
q_L5_1(i) = L5_x_rect_bot(i,1) - m_L5_1(i)*x_5_1(i,100);
L5_y_rect_1(i,:) = m_L5_1(i)*x_5_1(i,:) + q_L5_1(i);
m_L5_100(i) = (L5_x_rect_top(i,100) - L5_x_rect_bot(i,100))/(x_5_100(i,1) - x_5_100(i,100)); 
q_L5_100(i) = L5_x_rect_bot(i,100) - m_L5_100(i)*x_5_100(i,100);
L5_y_rect_100(i,:) = m_L5_100(i)*x_5_100(i,:) + q_L5_100(i);


x_1(i,:) = linspace(Oj1_t(i,1),EE1_t(i,1));
m_L1(i) = (EE1_t(i,2) - Oj1_t(i,2))/(EE1_t(i,1) - Oj1_t(i,1)); 
q_L1(i) = Oj1_t(i,2) - m_L1(i)*Oj1_t(i,1);
L1(i,:) = m_L1(i)*x_1(i,:) + q_L1(i);
x_1_top(i,:) = x_1(i,:) - (man_width(1)/2)*cos(-pi/4);
x_1_bot(i,:) = x_1(i,:) + (man_width(1)/2)*cos(-pi/4);
L1_x_rect_top(i,:) = L1(i,:) + (man_width(1)/2)*cos(-pi/4);
L1_x_rect_bot(i,:) = L1(i,:) - (man_width(1)/2)*cos(-pi/4);
x_1_1(i,:) = linspace(Oj1_t(i,1) - (man_width(1)/2)*cos(pi/4),Oj1_t(i,1) + (man_width(1)/2)*cos(pi/4));
x_1_100(i,:) = linspace(EE1_t(i,1) - (man_width(1)/2)*cos(pi/4),EE1_t(i,1) + (man_width(1)/2)*cos(pi/4));
m_L1_1(i) = (L1_x_rect_top(i,1) - L1_x_rect_bot(i,1))/(x_1_1(i,1) - x_1_1(i,100)); 
q_L1_1(i) = L1_x_rect_bot(i,1) - m_L1_1(i)*x_1_1(i,100);
L1_y_rect_1(i,:) = m_L1_1(i)*x_1_1(i,:) + q_L1_1(i);
m_L1_100(i) = (L1_x_rect_top(i,100) - L1_x_rect_bot(i,100))/(x_1_100(i,1) - x_1_100(i,100)); 
q_L1_100(i) = L1_x_rect_bot(i,100) - m_L1_100(i)*x_1_100(i,100);
L1_y_rect_100(i,:) = m_L1_100(i)*x_1_100(i,:) + q_L1_100(i);


x_2(i,:) = linspace(EE1_t(i,1),EE2_t(i,1));
m_L2(i) = (EE2_t(i,2) - EE1_t(i,2))/(EE2_t(i,1) - EE1_t(i,1)); 
q_L2(i) = EE1_t(i,2) - m_L2(i)*EE1_t(i,1);
L2(i,:) = m_L2(i)*x_2(i,:) + q_L2(i);
x_2_top(i,:) = x_2(i,:) - (man_width(2)/2)*cos(-pi/4);
x_2_bot(i,:) = x_2(i,:) + (man_width(2)/2)*cos(-pi/4);
L2_x_rect_top(i,:) = L2(i,:) + (man_width(2)/2)*cos(-pi/4);
L2_x_rect_bot(i,:) = L2(i,:) - (man_width(2)/2)*cos(-pi/4);
x_2_1(i,:) = linspace(EE1_t(i,1) - (man_width(2)/2)*cos(pi/4),EE1_t(i,1) + (man_width(2)/2)*cos(pi/4));
x_2_100(i,:) = linspace(EE2_t(i,1) - (man_width(2)/2)*cos(pi/4),EE2_t(i,1) + (man_width(2)/2)*cos(pi/4));
m_L2_1(i) = (L2_x_rect_top(i,1) - L2_x_rect_bot(i,1))/(x_2_1(i,1) - x_2_1(i,100)); 
q_L2_1(i) = L2_x_rect_bot(i,1) - m_L2_1(i)*x_2_1(i,100);
L2_y_rect_1(i,:) = m_L2_1(i)*x_2_1(i,:) + q_L2_1(i);
m_L2_100(i) = (L2_x_rect_top(i,100) - L2_x_rect_bot(i,100))/(x_2_100(i,1) - x_2_100(i,100)); 
q_L2_100(i) = L2_x_rect_bot(i,100) - m_L2_100(i)*x_2_100(i,100);
L2_y_rect_100(i,:) = m_L2_100(i)*x_2_100(i,:) + q_L2_100(i);


x_3(i,:) = linspace(EE2_t(i,1),EE3_t(i,1));
m_L3(i) = (EE3_t(i,2) - EE2_t(i,2))/(EE3_t(i,1) - EE2_t(i,1)); 
q_L3(i) = EE2_t(i,2) - m_L3(i)*EE2_t(i,1);
L3(i,:) = m_L3(i)*x_3(i,:) + q_L3(i);
x_3_top(i,:) = x_3(i,:) - (man_width(3)/2)*cos(-pi/4);
x_3_bot(i,:) = x_3(i,:) + (man_width(3)/2)*cos(-pi/4);
L3_x_rect_top(i,:) = L3(i,:) + (man_width(3)/2)*cos(-pi/4);
L3_x_rect_bot(i,:) = L3(i,:) - (man_width(3)/2)*cos(-pi/4);
x_3_1(i,:) = linspace(EE2_t(i,1) - (man_width(3)/2)*cos(pi/4),EE2_t(i,1) + (man_width(3)/2)*cos(pi/4));
x_3_100(i,:) = linspace(EE3_t(i,1) - (man_width(3)/2)*cos(pi/4),EE3_t(i,1) + (man_width(3)/2)*cos(pi/4));
m_L3_1(i) = (L3_x_rect_top(i,1) - L3_x_rect_bot(i,1))/(x_3_1(i,1) - x_3_1(i,100)); 
q_L3_1(i) = L3_x_rect_bot(i,1) - m_L3_1(i)*x_3_1(i,100);
L3_y_rect_1(i,:) = m_L3_1(i)*x_3_1(i,:) + q_L3_1(i);
m_L3_100(i) = (L3_x_rect_top(i,100) - L3_x_rect_bot(i,100))/(x_3_100(i,1) - x_3_100(i,100)); 
q_L3_100(i) = L3_x_rect_bot(i,100) - m_L3_100(i)*x_3_100(i,100);
L3_y_rect_100(i,:) = m_L3_100(i)*x_3_100(i,:) + q_L3_100(i);


x_4(i,:) = linspace(EE3_t(i,1),EE4_t(i,1));
m_L4(i) = (EE4_t(i,2) - EE3_t(i,2))/(EE4_t(i,1) - EE3_t(i,1)); 
q_L4(i) = EE3_t(i,2) - m_L4(i)*EE3_t(i,1);
L4(i,:) = m_L4(i)*x_4(i,:) + q_L4(i);
x_4_top(i,:) = x_4(i,:) - (man_width(4)/2)*cos(-pi/4);
x_4_bot(i,:) = x_4(i,:) + (man_width(4)/2)*cos(-pi/4);
L4_x_rect_top(i,:) = L4(i,:) + (man_width(4)/2)*cos(-pi/4);
L4_x_rect_bot(i,:) = L4(i,:) - (man_width(4)/2)*cos(-pi/4);
x_4_1(i,:) = linspace(EE4_t(i,1) - (man_width(4)/2)*cos(pi/4),EE3_t(i,1) + (man_width(4)/2)*cos(pi/4));
x_4_100(i,:) = linspace(EE4_t(i,1) - (man_width(4)/2)*cos(pi/4),EE4_t(i,1) + (man_width(4)/2)*cos(pi/4));
m_L4_1(i) = (L4_x_rect_top(i,1) - L4_x_rect_bot(i,1))/(x_4_1(i,1) - x_4_1(i,100)); 
q_L4_1(i) = L4_x_rect_bot(i,1) - m_L4_1(i)*x_4_1(i,100);
L4_y_rect_1(i,:) = m_L4_1(i)*x_4_1(i,:) + q_L4_1(i);
m_L4_100(i) = (L4_x_rect_top(i,100) - L4_x_rect_bot(i,100))/(x_4_100(i,1) - x_4_100(i,100)); 
q_L4_100(i) = L4_x_rect_bot(i,100) - m_L4_100(i)*x_4_100(i,100);
L4_y_rect_100(i,:) = m_L4_100(i)*x_4_100(i,:) + q_L4_100(i);


% Base motion -- for plotting
x_v_C0(i,:) = [- base_side/2, + base_side/2, + base_side/2, - base_side/2, - base_side/2];
y_v_C0(i,:) = [- base_side/2, - base_side/2, + base_side/2, + base_side/2, - base_side/2];

x_ruot_C0(i,:) = x_v_C0(i,:) * cos(thetas(i,1)) - y_v_C0(i,:) * sin(thetas(i,1));
y_ruot_C0(i,:) = x_v_C0(i,:) * sin(thetas(i,1)) + y_v_C0(i,:) * cos(thetas(i,1));

x_rot_C0(i,:) = x_ruot_C0(i,:) + C0_t(i,1);
y_rot_C0(i,:) = y_ruot_C0(i,:) + C0_t(i,2);

end




