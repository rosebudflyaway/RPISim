%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
% test_flat_NI.m
%
% Test a particle in simulation with the CDA and S-T methods, against
% a flat, level plane, non interactive mode



function test_flat_NI(h)


if (0 == nargin)
    h = 1e-4;
end

[B, V] = test_CDA_flat_init(h);
[B_ST, V_ST] = test_CDA_flat_init(h);
[B_AP, V_AP] = test_CDA_flat_init(h);


test_CDA_simulate_noninteractive(B, V, B_ST, V_ST, B_AP, V_AP);




