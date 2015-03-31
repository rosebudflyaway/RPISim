%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
% test_CDA_flat_init.m
%
% Initialize the body and vertex data for the flat plane test case

 
function [B, V] = test_CDA_flat_init(h)



% Define body edges here. 

% A single edge
B(1).x = [0 3];
B(1).y = [0 0];




% Define the particle
V.x = 0;  % Initial point
V.y = 0.5;
V.m = 5;    % mass
%  V.nu = [2.0; 1.8];  % velocity
V.nu = [2.0; 0.0];  % velocity
%  V.nu = [0; 0];

V.mu = 0.5;       % coefficient of friction
V.DEFINE_FRICTION = 1; % use friction
V.b = 0.0; % vicous force

V.grav = 9.81;
V.Fext = [0; -V.grav * V.m]; % a little reaction motor or something, plus gravity

V.T = 0.0;
V.h = h;

V.min_eps = 0.25;

V.max_steps = 1.5 / V.h;


%  V.U_datum = V.y;
V.U_datum = -1.0;
