

% INPUT:
%       bdy     - A Body object
%       edgeID  - Index of the tail vertex of an edge on bdy
%
%                         Example edge
%          tail vertex  *-------------->*  head vertex
% 
function n = edge_normal( bdy, edgeID )

    vh = bdy.verts_world(edgeID,:); 
    if edgeID == bdy.num_verts
        vt = bdy.verts_world(1,:);
    else
        vt = bdy.verts_world(edgeID+1,:); 
    end

    n = vt - vh;
    n = [ -n(2) n(1) ];  
    n = n/norm(n); 
    
end

