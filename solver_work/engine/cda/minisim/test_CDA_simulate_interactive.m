%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
% test_CDA_simulate_interactive.m
%
% Run an interactive mini-simulation

function test_CDA_simulate_interactive(B, V, method_str)

if (2 == nargin)
    method_str = 'S-T';
end


test_CDA_edges_plot(B, V);


avg_penetration_depth = 0.0;
T_remaining = 0.0;
psi_list = [];

while 1==1
    
    if T_remaining <= 0.0
        [x,y,button] = ginput(1); 
    end
    
    if button == 1
        V.x = x;
        V.y = y;

        V.nu = [0.5; 0];
        button = 0;
    end

    if button == 2
        % add a half second of iterations to run_count
        %run_count = floor(0.5 / h);
        T_remaining = 0.1; 
        button = 0;
    end

    if button == 3
        break; 
    end

    test_CDA_edges_plot(B, V);
    
    if ('CDA' == method_str)
        [V.nu, log_data] = test_CDA_dynamics(B, V);
    end
    
    if ('S-T' == method_str)
        [V.nu, log_data] = test_ST_dynamics(B, V);
    end

    if T_remaining > 0.0
        V.x = V.x + V.nu(1) * V.h;
        V.y = V.y + V.nu(2) * V.h;

        T_remaining = T_remaining - V.h;
        V.T = V.T + V.h;

    end
end


if (~isempty(psi_list))
    % Create plot of psi
    figure(2);
    set(gcf, 'Color', [1 1 1]);
    bar(psi_list);
else
%      fprintf(1, 'No psi values to plot.\n');
end
