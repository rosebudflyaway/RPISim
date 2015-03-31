%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
% test_CDA_calc_epsilon.m
%
% Calculate epsilon - the radius of the sphere of influence for setting up
% constraints


function this_eps = test_CDA_calc_epsilon(V)


% epsilon is a function of current velocity, approximate accel, and timestep

this_eps = norm(V(1).h * V(1).nu + V(1).h^2 * V(1).Fext / V(1).m);

if (isfield(V, 'min_eps'))
    if (this_eps < V(1).min_eps)
        this_eps = V(1).min_eps;
    end
end
    

