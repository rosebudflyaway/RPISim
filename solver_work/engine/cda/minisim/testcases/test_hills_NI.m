%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
% test_sawtooth_NI.m
%
% Test a particle in simulation with the CDA and S-T methods, against
% a series of hills, non interactive mode



function test_hills_NI(h)


if (0 == nargin)
    h = 1e-2;
end

[B, V] = test_CDA_hills_init(h);
[B_ST, V_ST] = test_CDA_hills_init(h);
[B_AP, V_AP] = test_CDA_hills_init(h);


test_CDA_simulate_noninteractive(B, V, B_ST, V_ST, B_AP, V_AP);


