%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
% test_CDAsawtooth_init.m
%
% Initialize the body and vertex data for the sawtooth simulation

 
function [B, V] = test_CDA_sawtooth_init(h)



% Define body edges here. 

% A single sawtooth (sharp edge)
sawtoothB.x = [0 2 2];
sawtoothB.y = [0 1 0];

% A single sawtooth (shallow edge)
%  sawtoothB.x = [0 1 2];
%  sawtoothB.y = [0 0.5 0];


% generate the teeth
num_bodies = 3;
xPos = 0.0;
for incr_b = 1:num_bodies
  B(incr_b).x = sawtoothB.x + xPos;
B(incr_b).y = sawtoothB.y;

xPos = xPos + (B(incr_b).x(length(B(incr_b).x)) - B(incr_b).x(1));
end



% Define the particle
V.x = 0;  % Initial point
V.y = 0.75;
V.m = 5;    % mass
%  V.nu = [2.0; 1.8];  % velocity
V.nu = [6.0; 1.8];  % velocity
%  V.nu = [0; 0];

V.mu = 0.5;       % coefficient of friction
V.DEFINE_FRICTION = 1; % use friction
V.b = 0.0; % vicous force

V.grav = 9.81;
V.Fext = [0; -V.grav * V.m]; % a little reaction motor or something, plus gravity

V.T = 0.0;
V.h = h;

V.min_eps = 0.0;

V.max_steps = 1.5 / V.h;


%  V.U_datum = V.y;
V.U_datum = -1.0;
