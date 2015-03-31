%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
% test_sawtooth.m
%
% Test a particle in simulation with the CDA method, against a series of 
% triangular bodies, interactive version


function test_sawtooth(h)


if (0 == nargin)
  h = 0.01; % step size
end



[B, V] = test_CDA_sawtooth_init(h);
test_CDA_simulate_interactive(B, V, 'CDA');
