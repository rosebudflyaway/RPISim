%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
% test_hills.m
%
% Test a particle in simulation with the CDA method, against a series of 
% rounded bodies, interactive version


function test_hills(h)

if (0 == nargin)
  h = 0.01; % step size
end

[B, V] = test_CDA_hills_init(h);
test_CDA_simulate_interactive(B, V, 'CDA');
