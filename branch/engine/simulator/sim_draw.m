%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
% sim_draw.m
%
% Draws a 3D scene from a simulation class.

function sim_draw( obj ) 

    figure(obj.GUI.SIM_FIG);
    
    %% Draw 
    for i=1:length(obj.P)       % Draw all objects
        if ~obj.P{i,1}.static   % Don't bother re-drawing static objects
            obj.P{i,1}.draw(); 
        end
    end
    
    %if obj.GUI.SIM_RUN || obj.GUI.SIM_RUN_iters > 0     % TODO: this is not the best way to implement the GUI...
        % Draw detected collisions between bodies 
        if obj.GUI.DRAW_COLLISIONS
            sim_draw_collisions( obj ); 
        elseif ~isempty(obj.GUI.Hg)   % Clear collisions
            obj.GUI.Hg(obj.GUI.Hg == 0) = [];
            delete(obj.GUI.Hg); 
        end

        % Draw joints
        if obj.GUI.DRAW_JOINTS
            for j=1:length(obj.Joints)
               obj.Joints(j).draw();    % TODO: Currently, won't update correctly when DRAW_JOINTS is disabled. 
            end
        end

        set(obj.GUI.hGUItitle,'String',['Solver time (s) at step ' num2str(obj.STEP) ' on ' num2str(length(obj.Contacts)) ' contacts: ' num2str(obj.timers.solver)]);
        %set(gca, 'Box', 'On');
        axis equal;
    %end
    drawnow

end % End Sim_Draw()



