%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
% test_CDA_calc_energy.m 
%
% Calculates the potential and kinetic energy in the system

function E = test_CDA_calc_energy(V)




% Calculate potential energy (using datum Ud)
E.U = V.m * V.grav * (V.y - V.U_datum);

% Calculate kinetic energy
E.T = V.m * (V.nu(1)^2 + V.nu(2)^2) / 2;


