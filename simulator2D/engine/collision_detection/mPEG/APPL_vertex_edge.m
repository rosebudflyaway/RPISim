

% Inputs:
%           A   - Body struct
%           vi  - Vertex index of body A
%           B   - Body struct
%           ei  - "Edge" index of body B (really a vertex index)
%
% Output:
%           appl - A number representing the applicability of vertex vi
%           against edge ei. 

function appl = APPL_vertex_edge( A, vi, B, ei )

    % IN 2D, each vertex has two incoming edges, each with its own
    % applicability.  The applicability of the vertex is taken to be the
    % minimum applicability of these two.  
    
    % Determine the normal of edge ei
    if ei == B.num_verts
        E = B.verts_world(1,:) - B.verts_world(ei,:); 
    else
        E = B.verts_world(ei+1,:) - B.verts_world(ei,:); 
    end
    nn = [ -E(2) E(1) ];  % n is the negative edge normal of E
    nn = nn/norm(nn); 
    
    % Get edges connected to vi, coming toward vi
    if vi == A.num_verts
        E1 = A.verts_world(vi,:) - A.verts_world(1,:);
        E2 = A.verts_world(vi,:) - A.verts_world(vi-1,:);
    elseif vi == 1
        E1 = A.verts_world(vi,:) - A.verts_world(vi+1,:);
        E2 = A.verts_world(vi,:) - A.verts_world(A.num_verts,:);
    else
        E1 = A.verts_world(vi,:) - A.verts_world(vi+1,:);
        E2 = A.verts_world(vi,:) - A.verts_world(vi-1,:);
    end
    E1 = E1/norm(E1);
    E2 = E2/norm(E2); 
    
    appl = min( nn(1)*E1(1)+nn(2)*E1(2) , nn(1)*E2(1)+nn(2)*E2(2) );
    
end

