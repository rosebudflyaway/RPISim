%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
%
% Inputs:
%           sim     - A Simulation object with at least jointIndex joints.
%           j       - An integer index of a joint
% Ouput:    
%           sim     - The simulation object with updated joint j

function sim = updateJoint( sim, j )

    % VECTOR SCALE
    vscale = 0.1; 

    body1 = sim.bodies(sim.joints(j).body1id);  
    body2 = sim.bodies(sim.joints(j).body2id); % TODO: if there is only one body, then this will break

    % Update body joint frames
    R1 = [cos(body1.rot) -sin(body1.rot)
          sin(body1.rot)  cos(body1.rot) ];
    sim.joints(j).P1 = body1.pos + R1 * sim.joints(j).jointPos1;
    sim.joints(j).X1 = sim.joints(j).P1 + R1*vscale*[1;0];
    sim.joints(j).Y1 = sim.joints(j).P1 + R1*vscale*[0;1];

    R2 = [cos(body2.rot) -sin(body2.rot)
          sin(body2.rot)  cos(body2.rot) ];
    sim.joints(j).P2 = body2.pos + R2 * sim.joints(j).jointPos2; 
    sim.joints(j).X2 = sim.joints(j).P2 + R2*vscale*[1;0];
    sim.joints(j).Y2 = sim.joints(j).P2 + R2*vscale*[0;1];

    % Determine the current value of theta
    if sim.joints(j).jntCode == 2  % Revolute
       sim.joints(j).theta = body2.rot - body1.rot; 
    elseif sim.joints(j).jntCode == 3 % Prismatic
       sim.joints(j).theta = norm(sim.joints(j).P2 - sim.joints(j).P1); 
    else
       %error('This joint type is currently not implemented.  I am sorry!');
    end

end






