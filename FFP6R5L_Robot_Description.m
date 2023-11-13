% FINAL PROJECT --- GROUP 18
%
% Federico Mustich, Lorenzo Porpiglia, Gaetana Gaia Spanò, Vincenzo Trentacapilli
%
% This script creates the data structure "robot" which is the input of
% FF6R5L_Robot_Simulate. In the scriopt the user will insert the mass and
% the lenght of the base link (which is a square) and the masses, lenghts
% and widths of the links, which are all identical. It's possible for the
% user to set an initial inclination wrt z-axis of the manipulator.



clc
clear all
close all



%--- Manipulator Description a la DH ---%    
disp(['Insert base link mass [kg]',':'])
m0 = input('m0:');
m = zeros(1,5);
disp(['Insert links masses [kg]',':'])
m = ones(1,5)*input('mass:');
   

disp(['Insert base link lenght [m]',':'])
l0 = input('l0:');
l = zeros(1,5);
s = zeros(1,5);
disp(['Insert links lenghts and widths [m]',':'])
l = ones(1,5)*input('lenght:');
s = ones(1,5)*input('width:');


base_side = 2*l0;
man_side = 2*l;
man_width = s;


%Number of joints  --- specified for the software
data.n = 5;

%Base-spacecraft inertia matrix
data.base.mass = m0;
data.base.I = diag([1,1,data.base.mass/6*(base_side^2)]);   %In CCS of the link base

%Firts joint location with respect to base
data.base.T_L0_J1 = [eye(3),[+base_side/2;0;0];zeros(1,3),1];   %Homogenous Transixtion matrix from axes J1 to axes L0


% Joint w.r.t. the homogenous manipulator 
for i = 1 : 5
     data.man(i).type = 1; % Revolute Joint
end

%Joints
for i = 1 : 5
data.man(i).DH.d = 0;                       % Displacement along z-axis [m]
data.man(i).DH.alpha = 0;                   % Angular velocity along the z-axis [rad/s]
data.man(i).DH.a = man_side(i);             % Manipulators's lengths [m]
data.man(i).b = [data.man(1).DH.a/2;0;0];   % Coordinates of the COM C_i w.r.t O_ji [m]
data.man(i).mass = m(i);                    % Mass of the manipulators [kg]

%NOTE: WE NEED TO GIVE NON SINGULAR MATRIX!
data.man(i).I = diag([1,1,data.man(i).mass/12*(man_side(i)^2 + man_width(i)^2)]);   %Inertia matrix of the manipulator i
end

disp('Insert initial manipulator inclination in rad :')
data.man(1).DH.theta = input('Inclination:');       % Initial angle of rotation around z-axis [rad]
for i = 2 : 5
    data.man(i).DH.theta = 0;   % Initial angle of rotation around z-axis [rad]
end

%End-Effector 
data.EE.theta = 0;  %Rotation around z-axis [rad]
data.EE.d = 0;      %Translation along z-axis [m]

%--- Create robot structure ---%
[robot,TEE_Ln] = DH_Serial2robot(data);   %go from the parameters of description of the manipulator to the paremeters we expect


