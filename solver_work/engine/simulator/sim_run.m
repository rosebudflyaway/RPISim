%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
% sim_run.m
%  

function sim_run( sim )

    while sim.STEP < sim.MAX_ITERS
       
        % Step
        if sim.GUI.SIM_RUN
            sim.step();
        elseif sim.GUI.SIM_RUN_iters > 0
            sim.step();
            sim.GUI.SIM_RUN_iters = sim.GUI.SIM_RUN_iters - 1;  % Could potentially run > 1 step
            % E.g. user input '5' could set 
        end
       
        % Draw
        sim_draw( sim ); 
        
        if sim.GUI.SIM_QUIT
            break;
        end
       
    end

end % End sim_run()

