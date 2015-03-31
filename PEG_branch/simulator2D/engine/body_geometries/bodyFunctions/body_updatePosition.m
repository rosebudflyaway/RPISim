
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Kinematic update with or without velocity
% Update the position and orientation of a body, including world coordinates and face normals.
% INPUTS: 
%       B       - A Body_mesh struct to be updated. 
%       h       - The timestep size in seconds.  
%       newNu   - (optional) The body's velocity at the end of the current timestep.

function B = body_updatePosition( B, h, varargin )

    % Do not update static bodies
    %if ~B.dynamic, return; end

    % If there is a newNu assigned
    if nargin > 2
        newNu = varargin{1}; 
        % Velocity 
        B.nu = newNu;  

        % Translation
        B.pos = B.pos + h*newNu(1:2);   

        % Rotation 
        B.rot = B.rot + h*newNu(3); 
        
    % Update body purely by external force Fext
    else
        % Velocity 
        B.nu = B.nu + (B.Fext/B.mass)*h;   

        % Translation
        B.pos = B.pos + h*B.nu(1:2);   

        % Rotation 
        B.rot = B.rot + h*B.nu(3); 
    end
    
end























