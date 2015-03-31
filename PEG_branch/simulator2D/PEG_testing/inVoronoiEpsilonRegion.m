


% Determines if the point p is inside the region of the feature of B
function inRegion = inVoronoiEpsilonRegion( p, bodyB, vertexIndex )

    eps_vert = 10^-3;  
    theta = 0.4;  
    
    v1 = bodyB.verts_world(vertexIndex,:);
    if vertexIndex == bodyB.num_verts
        v2 = bodyB.verts_world(1,:);
    else
        v2 = bodyB.verts_world(vertexIndex+1,:);
    end
    
    edge = v2-v1;
    edge = edge/norm(edge); 
    
    n = v1-v2;
    n = [-n(2) n(1)];
    n = n/norm(n); 
    
    n1 = [cos(-theta) -sin(-theta)
          sin(-theta)  cos(-theta) ] * n';
    n2 = [cos(theta) -sin(theta)
          sin(theta)  cos(theta) ] * n';
    
    if dot( p-(v1+eps_vert*edge) , n1' ) < 0  && dot( p-(v2-eps_vert*edge) , n2' ) < 0
        inRegion = true;
    else
        inRegion = false; 
    end

end

