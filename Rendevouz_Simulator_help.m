% FINAL PROJECT --- GROUP 18
%
% Federico Mustich, Lorenzo Porpiglia, Gaetana Gaia Spanò, Vincenzo Trentacapilli
%
% This script simulates the rendezvous between a chief and a deputy in a
% circular orbit around the Earth. The deputy is initially setted at a
% random position and velocity. The chief is considered fixed at the origin
% of our CCS. The deputy's trajectory is computed using the Clohessy-Wiltshire
% equations. The user can control the deputy's trajectory using the keyboard.
% The goal is to reach the chief's position (with a tolerance of 0.5 m) with a
% safe velocity (with a tolerance of 0.01 m/s). The user can also
% pause the simulation and exit it or adjust the time rate of the simulation.
%
% COMMANDS:
%   - W: increase the deputy's velocity along the x-axis (Prograde acceleration)
%   - S: decrease the deputy's velocity along the x-axis (Retrograde acceleration)
%   - A: increase the deputy's velocity along the y-axis (Normal acceleration)
%   - D: decrease the deputy's velocity along the y-axis (Anti-normal acceleration)
%   - Q: increase the deputy's velocity along the z-axis (Radial acceleration)
%   - E: decrease the deputy's velocity along the z-axis (Anti-radial acceleration)
%   - P: pause the simulation
%   - T: increase the time rate of the simulation
%   - G: decrease the time rate of the simulation
%   - SPACE: pause the simulation
%   - X: exit the simulation