%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
% test_CDA_hills_init.m
%
% Initialize the body and vertex data for the hills simulation

 
function [B, V] = test_CDA_hills_init(h)



% Define body edges here. 
%%%%%%%%%%%%%%%%%%%%%%%%%

% each hill is a hemisphere, x = cos(u), y = sin(u) where u = 0:pi

% Define these parameters:
num_bodies = 2; % number of hills
num_edges = 7; % number of edges per hill
%  num_bodies = 1;
%  num_edges = 3;
r = 1; % radius of hill, in meters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


ustep = pi / num_edges;

u = pi:-ustep:0;

% A single hill
hillB.x = cos(u);
hillB.y = sin(u);



% generate the hills

xPos = 0.0;
for incr_b = 1:num_bodies
    B(incr_b).x = hillB.x + xPos;
    B(incr_b).y = hillB.y;

    xPos = xPos + (B(incr_b).x(length(B(incr_b).x)) - B(incr_b).x(1));
end



% Define the particle
V.x = 0;  % Initial point
V.y = 1.5;
V.m = 5;    % mass
V.nu = [7.0; 0.0];  % velocity

V.DEFINE_FRICTION = 1;
V.mu = 0.5; % coefficient of friction
V.b = 0.5; % vicous force

V.grav = 9.81;
V.Fext = [0.0; -V.grav * V.m]; % a little reaction motor or something, plus gravity

V.T = 0.0;
V.h = h;

V.min_eps = 0.5;

V.max_steps = 500;

V.U_datum = 0.0;