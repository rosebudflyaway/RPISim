

% Assumes the vertex p is on the interior of bodyB
function [ EB N PSI ] = nearestEdgeDistance( p, bodyB )

    B = bodyB.verts_world;
    
    % Last edge first
    eb = B(1,:) - B(end,:);
    nb = [eb(2) -eb(1)]; 
    nb = nb/norm(nb);
    psi = dot(p-B(1,:),nb);
    
    EB = bodyB.num_verts;
    PSI= psi;
    N = nb;
    
    % All other edges
    for i=2:bodyB.num_verts
       eb = B(i,:) - B(i-1,:); 
       nb = [eb(2) -eb(1)]; 
       nb = nb/norm(nb);
       psi = dot(p-B(i,:),nb);
       
       if psi > PSI %abs(psi) > abs(PSI)
           EB = i-1;
           PSI = psi; 
           N = nb;
       end
    end
    
end

