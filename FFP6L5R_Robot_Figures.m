% FINAL PROJECT --- GROUP 18
%
% Federico Mustich, Lorenzo Porpiglia, Gaetana Gaia Spanò, Vincenzo Trentacapilli
%
% This script plots : figures of the most important quantities for one quarter of an orbit, 
% and figures showing the conservation of momentum and angular momentum up to numerical accuracy.
% At last an animation of the system for a certain propagational time shows
% up.


clear all
clc
close all

 FFP6R5L_Robot_Simulate

sysname='FFP_6L_5R';
t = t_out;
% There are six rotational DOFs because these are theta_0, theta_1, theta_2, theta_3, theta_4 and  theta_6
num_rotational_dof = 6;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure('Name',strcat(sysname,'_NSysCOM'),'DefaultAxesFontSize',16)
fprintf('\n')
fprintf('\n')
fprintf('***********************\n')
fprintf(strcat('CAPTION OF Figure ', sysname,'_NSysCOM:\n'))
fprintf('Evolution in time of the coordinates of the system CoM on the N CCS\n') 
fprintf('each subplot is showing a gen. coordinate in darker line and the corresponding generalized velocity in lighter line.\n')
fprintf('***********************\n')
fprintf('\n')
fprintf('\n')
subplot(1,2,1)
plot(t,SCMt(:,1),'LineWidth', 2, 'color','k')
xlabel('t [s]')
ylabel('x_{SCM} on the N CCS [m]')
subplot(1,2,2)
plot(t,SCMt(:,2),'LineWidth', 2, 'color','k')
grid minor
xlabel('t [s]')
ylabel('y_{SCM} on the N CCS [m]')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure('Name',strcat(sysname,'_NSysMoMwrtC0'),'DefaultAxesFontSize',16)
fprintf('\n')
fprintf('\n')
fprintf('***********************\n')
fprintf(strcat('CAPTION OF Figure ', sysname,'_NSysMoMwrtC0:\n'))
fprintf('Evolution in time of the Moment of Momentum w.r.t. C0 (CoM L0)\n') 
fprintf('***********************\n')
fprintf('\n')
fprintf('\n')
plot(t,MoMt_C0(:,3)-MoMt_C0(1,3)*ones(size(MoMt_C0)),'LineWidth', 2, 'color','k')
grid minor
xlabel('t [s]')
ylabel('MoM wrt C0 minus its Init. Value [Nm]')
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure('Name',strcat(sysname,'_NSysLinearMomwrtC0'),'DefaultAxesFontSize',16)
fprintf('\n')
fprintf('\n')
fprintf('***********************\n')
fprintf(strcat('CAPTION OF Figure ', sysname,'_NSysLinearMomwrtC0:\n'))
fprintf('Evolution in time of the Linear Momentum w.r.t. C0 (CoM L0)\n') 
fprintf('***********************\n')
fprintf('\n')
fprintf('\n')
plot(t,Linear_Mom_CO(:,1)-Linear_Mom_CO(1,1)*ones(size(Linear_Mom_CO)),'LineWidth', 2, 'color','k')
grid minor
xlabel('t [s]')
ylabel('X Linear Momentum component wrt C0 minus its Init. Value [Nm]')
% % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure('Name',strcat(sysname,'_NSysLinearMomwrtC0'),'DefaultAxesFontSize',16)
fprintf('\n')
fprintf('\n')
fprintf('***********************\n')
fprintf(strcat('CAPTION OF Figure ', sysname,'_NSysLinearMomwrtC0:\n'))
fprintf('Evolution in time of the Linear Momentum w.r.t. C0 (CoM L0)\n') 
fprintf('***********************\n')
fprintf('\n')
fprintf('\n')
plot(t,Linear_Mom_CO(:,2)-Linear_Mom_CO(1,2)*ones(size(Linear_Mom_CO)),'LineWidth', 2, 'color','k')
grid minor
xlabel('t [s]')
ylabel('Y Linear Momentum component wrt C0 minus its Init. Value [Nm]')
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure('Name',strcat(sysname,'_NSysMoMwrtOI'),'DefaultAxesFontSize',16)
fprintf('\n')
fprintf('\n')
fprintf('***********************\n')
fprintf(strcat('CAPTION OF Figure ', sysname,'_NSysMoMwrtOI:\n'))
fprintf('Evolution in time of the Moment of Momentum w.r.t. OI (Origin of Inertially Fixed CCS)\n')
fprintf('***********************\n')
fprintf('\n')
fprintf('\n')
plot(t, MoMt_OI(:,3)-MoMt_OI(1,3)*ones(size(MoMt_OI)),'LineWidth', 2, 'color','k')
grid minor
xlabel('t [s]')
ylabel('MoM wrt OI minus its Init. Value [Nm]')
legend('MoM wrt OI on L_1,L_2,L_3,L_4,L_5',Location='best')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure('Name',strcat(sysname,'_NSysMoMwrtC'),'DefaultAxesFontSize',16)
fprintf('\n')
fprintf('\n')
fprintf('***********************\n')
fprintf(strcat('CAPTION OF Figure ', sysname,'_NSysMoMwrtC:\n'))
fprintf('Evolution in time of the Moment of Momentum w.r.t. C (system CoM)\n')
fprintf('***********************\n')
fprintf('\n')
fprintf('\n')
plot(t, MoMt_sysC(:,3)-MoMt_sysC(1,3)*ones(size(MoMt_sysC)),'LineWidth', 2, 'color','k')
grid minor
xlabel('t [s]')
ylabel('MoM wrt C minus its Init. Value [Nm]')
legend('MoM wrt C on L_1,L_2,L_3,L_4,L_5',Location='best')
figure('Name',strcat(sysname,'_NAxialMoMmwrtj1'),'DefaultAxesFontSize',16)
fprintf('\n')
fprintf('\n')
fprintf('***********************\n')
fprintf(strcat('CAPTION OF Figure ', sysname,'_NAxialMoMmwrtj_i:\n'))
fprintf('Evolution in time of the Axial Moment of Momentum of the manipulator w.r.t. j1 (joint 1)\n')
fprintf('***********************\n')
fprintf('\n')
fprintf('\n')
plot(t, Axial_MoMm_j1(:,1),'LineWidth', 2, 'color','k')
hold on
plot(t, Axial_MoMm_j2(:,1),'LineWidth', 2, 'color','r')
plot(t, Axial_MoMm_j3(:,1),'LineWidth', 2, 'color','y')
plot(t, Axial_MoMm_j4(:,1),'LineWidth', 2, 'color','g')
plot(t, Axial_MoMm_j5(:,1),'LineWidth', 2, 'color','b')
grid minor
legend ('Axial_MoMm_j1','Axial_MoMm_j2','Axial_MoMm_j3','Axial_MoMm_j4','Axial_MoMm_j5')
xlabel('t [s]')
ylabel('Axial MoMm at j1, j2, j3, j4, j5')
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure('Name',strcat(sysname,'_thetas_dot_thetas'),'DefaultAxesFontSize',16)
fprintf('\n')
fprintf('\n')
fprintf('***********************\n')
fprintf(strcat('CAPTION OF Figure ', sysname,'_thetas_dot_thetas:\n'))
fprintf('Evolution in time of the theta coordinates (theta0, and theta1) and velocites \n') 
fprintf('each subplot is showing a theta coordinate in darker line and the corresponding generalized velocity in lighter line.\n')
fprintf('***********************\n')
fprintf('\n')
fprintf('\n')
for ii = 1:num_rotational_dof
    subplot(num_rotational_dof,1,ii)
    plot(t,thetas(:,ii),'LineWidth', 2, 'color','k')
    hold on
    grid minor
    plot(t,dot_thetas(:,ii),'LineWidth', 0.5, 'color','r')
    xlabel('t [s]')
    ylabel(strcat('$\theta$',num2str(ii),', $\dot{\theta}$',num2str(ii)),'Interpreter','latex')
end
sgtitle('theta [rad] - dot theta [rad/s]');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 


for i = 1 : 15 : length(t_out) 

% figure('Name',strcat(sysname,'_BirdView'),'DefaultAxesFontSize',16)
figure(9)

fprintf('\n')
fprintf('\n')
fprintf('***********************\n')
fprintf(strcat('CAPTION OF Figure ', sysname,'_BirdView:\n'))
fprintf('Evolution on plane of motion\n')
fprintf('***********************\n')
fprintf('\n')
fprintf('\n')

plot(SCMt(:,1),SCMt(:,2),'LineWidth', 2, 'color','k','DisplayName','CoM')
hold on
txt = '\leftarrow C at t_0';
text(SCMt(1,1),SCMt(1,2),txt,'FontSize',10) 

plot(C0_t(:,1),C0_t(:,2),'LineWidth', 2, 'color','k','DisplayName','C0')
txt = '\leftarrow C0 at t_0';
text(C0_t(1,1),C0_t(1,2),txt,'FontSize',10,'color','k' ) 

plot(C1_t(:,1),C1_t(:,2),'-','LineWidth', 0.5, 'color','k','DisplayName','C0')
txt = '\leftarrow C1 at t_0';
text(C1_t(1,1),C1_t(1,2),txt,'FontSize',10,'color','k') 
plot(Oj1_t(:,1),Oj1_t(:,2),'-','LineWidth', 0.5, 'color','k','DisplayName','C0')
txt = '\leftarrow Oj1 at t_0';
text(Oj1_t(1,1),Oj1_t(1,2),txt,'FontSize',10,'color','k') 

plot(C2_t(:,1),C2_t(:,2),'-','LineWidth', 0.5, 'color','k','DisplayName','C0')
txt = '\leftarrow C2 at t_0';
text(C2_t(1,1),C2_t(1,2),txt,'FontSize',10,Color='k')
plot(Oj2_t(:,1),Oj2_t(:,2),'-','LineWidth', 0.5, 'color','k','DisplayName','C0')
txt = '\leftarrow Oj2 at t_0';
text(Oj2_t(1,1),Oj2_t(1,2),txt,'FontSize',10,Color='k')


plot(C3_t(:,1),C3_t(:,2),'-','LineWidth', 0.5, 'color','k','DisplayName','C0')
txt = '\leftarrow C3 at t_0';
text(C3_t(1,1),C3_t(1,2),txt,'FontSize',10,Color='k') 
plot(Oj3_t(:,1),Oj3_t(:,2),'-','LineWidth', 0.5, 'color','k','DisplayName','C0')
txt = '\leftarrow Oj3 at t_0';
text(Oj3_t(1,1),Oj3_t(1,2),txt,'FontSize',10,Color='k') 

plot(C4_t(:,1),C4_t(:,2),'-','LineWidth', 0.5, 'color','k','DisplayName','C0')
txt = '\leftarrow C4 at t_0';
text(C4_t(1,1),C4_t(1,2),txt,'FontSize',10,Color='k')
plot(Oj4_t(:,1),Oj4_t(:,2),'-','LineWidth', 0.5, 'color','k','DisplayName','C0')
txt = '\leftarrow Oj4 at t_0';
text(Oj4_t(1,1),Oj4_t(1,2),txt,'FontSize',10,Color='k')

plot(C5_t(:,1),C5_t(:,2),'-','LineWidth', 0.5, 'color','k','DisplayName','C0')
txt = '\leftarrow C5 at t_0';
text(C5_t(1,1),C5_t(1,2),txt,'FontSize',10,Color='k')
plot(Oj5_t(:,1),Oj5_t(:,2),'-','LineWidth', 0.5, 'color','k','DisplayName','C0')
txt = '\leftarrow Oj5 at t_0';
text(Oj5_t(1,1),Oj5_t(1,2),txt,'FontSize',10,Color='k')

plot(EE5_t(:,1),EE5_t(:,2),'-','LineWidth', 0.5, 'color','k','DisplayName','C0')
txt = '\leftarrow EE at t_0';
text(EE5_t(1,1),EE5_t(1,2),txt,'FontSize',10,Color='k')

    plot(SCMt(i,1),SCMt(i,2),'.','MarkerSize',24, 'color','c','DisplayName','CoM')
    plot(C0_t(i,1),C0_t(i,2),'.','MarkerSize',24, 'color','g','DisplayName','C0')
    plot(Oj1_t(i,1),Oj1_t(i,2),'.','MarkerSize',24, 'color','r','DisplayName','C0')
    plot(Oj2_t(i,1),Oj2_t(i,2),'.','MarkerSize',24, 'color','r','DisplayName','C0')
    plot(Oj3_t(i,1),Oj3_t(i,2),'.','MarkerSize',24, 'color','r','DisplayName','C0')
    plot(Oj4_t(i,1),Oj4_t(i,2),'.','MarkerSize',24, 'color','r','DisplayName','C0')
    plot(Oj5_t(i,1),Oj5_t(i,2),'.','MarkerSize',24, 'color','r','DisplayName','C0')
    plot(EE5_t(i,1),EE5_t(i,2),'.','MarkerSize',24, 'color','r','DisplayName','C0')


    plot(x_rot_C0(i,:),y_rot_C0(i,:),'LineWidth', 1.5, 'color','k','DisplayName','C0')

    plot(x_1_top(i,:),L1_x_rect_top(i,:),'LineWidth', 1, 'color','k','DisplayName','C0')
    plot(x_1_bot(i,:),L1_x_rect_bot(i,:),'LineWidth', 1, 'color','k','DisplayName','C0')
    plot(x_1_1(i,:),L1_y_rect_1(i,:),'LineWidth', 1, 'color','k','DisplayName','C0')
    plot(x_1_100(i,:),L1_y_rect_100(i,:),'LineWidth', 1, 'color','k','DisplayName','C0')
    plot(x_2_top(i,:),L2_x_rect_top(i,:),'LineWidth', 1, 'color','k','DisplayName','C0')
    plot(x_2_bot(i,:),L2_x_rect_bot(i,:),'LineWidth', 1, 'color','k','DisplayName','C0')
%     plot(x_2_1(i,:),L2_y_rect_1(i,:),'LineWidth', 1, 'color','k','DisplayName','C0')
    plot(x_2_100(i,:),L2_y_rect_100(i,:),'LineWidth', 1, 'color','k','DisplayName','C0')
    plot(x_3_top(i,:),L3_x_rect_top(i,:),'LineWidth', 1, 'color','k','DisplayName','C0')
    plot(x_3_bot(i,:),L3_x_rect_bot(i,:),'LineWidth', 1, 'color','k','DisplayName','C0')
%     plot(x_3_1(i,:),L3_y_rect_1(i,:),'LineWidth', 1, 'color','k','DisplayName','C0')
    plot(x_3_100(i,:),L3_y_rect_100(i,:),'LineWidth', 1, 'color','k','DisplayName','C0')
    plot(x_4_top(i,:),L4_x_rect_top(i,:),'LineWidth', 1, 'color','k','DisplayName','C0')
    plot(x_4_bot(i,:),L4_x_rect_bot(i,:),'LineWidth', 1, 'color','k','DisplayName','C0') 
    plot(x_5_top(i,:),L5_x_rect_top(i,:),'LineWidth', 1, 'color','k','DisplayName','C0')
    plot(x_5_bot(i,:),L5_x_rect_bot(i,:),'LineWidth', 1, 'color','k','DisplayName','C0')
    
    plot(x_5_1(i,:),L5_y_rect_1(i,:),'LineWidth', 1, 'color','k','DisplayName','C0')
    plot(x_5_100(i,:),L5_y_rect_100(i,:),'LineWidth', 1, 'color','k','DisplayName','C0')

    hold off
    pause(0.001);
end
grid minor
axis([-base_side*2 base_side*4 -base_side*2 base_side*4]);
xlabel('x [m]')
ylabel('y [m]')
hold off

