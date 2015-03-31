%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
% kinematic_update.m
%
% Assumes that the dynamics solution has update obj.z with the active
% bodies' new velocities.  

function kinematic_update( sim )

    h = sim.h; 

    % Apply results from solution, as well as update bodies not in contact.   
    for i = 1 : length(sim.P)     
       body = sim.P{i};  
       if body.static, continue; end;   % Don't update static bodies. 

       if body.ContactCount <= 0        % Body had no constraints (apply external forces)
            body.kinematicUpdate(h);
       else                             % Body had a constraint
            bodyID = body.BodyIndex; 
            body.kinematicUpdateWithVelocity(sim.z(6*bodyID-5:6*bodyID), h);
       end
    end

end

