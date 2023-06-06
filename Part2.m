clc
clear all
close all
disp('0: Fixed Joint')
disp('1: Revolute Joint')
disp('2: Prismatic Joint')
Joint=input('Please, insert the desired option: ');


%--- Manipulator Description a la DH ---%    
% disp(['Insert base link mass [kg]',':'])
% m0 = input('m0:');
% m = zeros(1,5);
% for i = 1 : 5
%     disp(['Insert link',num2str(i),' mass [kg]',':'])
%     m(i) =input('mass: ');
% end
% 
% %
% disp(['Insert base link lenght [m]',':'])
% l0 = input('l0:');
% l = zeros(1,5);
% s = zeros(1,5);
% for i = 1 : 5
%     disp(['Insert link', num2str(i),' lenghts and widths [m]',':'])
%     l(i) =input('lenght:');
%     s(i) =input('width:');
% end

m0=100;
% for i = 1:5
%     m(i) = 5;
% end
 l0=0.25;
for i = 1 : 5
%     disp(['Insert link', num2str(i),' lenghts and widths [m]',':'])
   
%     l(i) = 0.25 + l(i)/5;
    s(i) = 0.025;
end
l(1) = 0.4;
l(2) = 0.4;
l(3) = 0.4;
l(4) = 0.4;
l(5) = 0.4;

m(1) = 5;
m(2) = 6;
m(3) = 7;
m(4) = 8;
m(5) = 9;

base_side = 2*l0;
man_side = 2*l;
man_width = s;


%Number of joints  --- specified for the software
data.n=5;

%Base-spacecraft inertia matrix
data.base.mass=m0;
data.base.I=diag([1,1,data.base.mass/6*(base_side^2)]);   %In CCS of the link base

%Firts joint location with respect to base
data.base.T_L0_J1=[eye(3),[+base_side/2;0;0];zeros(1,3),1];   %Homogenous Transixtion matrix from axes J1 to axes L0


% Joint w.r.t. the homogenous manipulator 
for i = 1 : 5
    if Joint==0
        data.man(i).type=0; % Fixed Joint
    end
    if Joint==1
        data.man(i).type=1; % Revolute Joint
    end
    if Joint==2
        data.man(i).type=2; % Prismatic Joint
    end
end

%Joints
for i = 1 : 5
data.man(i).DH.d = 0; %displacement along z-axis
data.man(i).DH.alpha = 0; % Angular velocity along the z-axis
data.man(i).DH.a = man_side(i); % manipulators' lengths
data.man(i).DH.theta = 0; % Initial angle of rotation around z-axis
data.man(i).b = [data.man(1).DH.a/2;0;0];% Coordinates of the COM C_i w.r.t O_ji
data.man(i).mass = m(i); %mass of the manipulators 

%NOTE: WE NEED TO GIVE NON SINGULAR MATRIX!
data.man(i).I=diag([1,1,data.man(i).mass/12*(man_side(i)^2+man_width(i)^2)]);   %Inertia matrix of the manipulator i
end

%End-Effector 
data.EE.theta=0; %Rotation around z-axis
data.EE.d=0; %Translation along z-axis

%--- Create robot structure ---%
[robot,TEE_Ln] = DH_Serial2robot(data);   %go from the parameters of description of the manipulator to the paremeters we expect


%INPUT OF PART2_2