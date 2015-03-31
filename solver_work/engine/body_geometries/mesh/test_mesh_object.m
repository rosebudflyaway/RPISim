%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
% test_CDA.m
%  
% Draws a mesh for inspection.  In particular, we are looking at face 
%   normals to confirm that faces were properly defined in .poly files.  
%

function test_mesh_object( m )
    
    % Draw m
    m.color = [.8 .1 .1];
    m.draw; 
    axis equal;  
    hold on; 
    
    % Draw face normals
    for f = 1:m.num_faces
        
        % Find center of face
        V = [];
        for v=1:m.faces(f).num_verts
            V = [V m.verts(m.faces(f).verts(v)).local_coords];
            u = mean(V,2); 
        end
        
        % Plot face normal
        drawvec(u, u+m.faces(f).normal);
        
        % Check face (assumes the normal will be away from center)
        if dot( (u-[0;0;0]) , m.faces(f).normal) < 0
           disp(['Check face ' num2str(f)]); 
        end
    end

end

