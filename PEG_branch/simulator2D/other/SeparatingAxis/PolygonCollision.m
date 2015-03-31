
function [ colliding, C ] = PolygonCollision( polygonA, polygonB )

    function edgeVector = getEdge (polyBody, edgeDex)
       if edgeDex == polyBody.num_verts  
           edgeVector = polyBody.verts_world(1,:) - polyBody.verts_world(edgeDex,:);
       else
           edgeVector = polyBody.verts_world(edgeDex+1,:) - polyBody.verts_world(edgeDex,:);
       end
    end

    colliding = true; 
    C = []; 
    result = PolygonCollisionResult;
    result.Intersect = true;
    result.WillIntersect = true;
    
    edgeCountA = polygonA.num_verts;
    edgeCountB = polygonB.num_verts; 
    minIntervalDistance = inf;
    translationAxis = [0;1];
    minEdgeIndex = 1;
    
    for edgeIndex = 1 : edgeCountA+edgeCountB 
       if edgeIndex <= edgeCountA
           edge = getEdge(polygonA, edgeIndex); 
       else
           edge = getEdge(polygonB, edgeIndex-edgeCountA);   
       end
        
       %% Find if the polygons are currently intersecting
       
       % Find the axis perpendicular to the current edge
       axis = [-edge(2), edge(1)];
       axis = axis / norm(axis); 
       
       % Find the projection of the polygon on the current axis
       [minA, maxA] = ProjectPolygon(axis, polygonA); 
       [minB, maxB] = ProjectPolygon(axis, polygonB);
       
       % Check if the polygon projects are currently intersecting
       intervalDistance = IntervalDistance(minA, maxA, minB, maxB);
       if intervalDistance > 0
           %result.Intersect = false;    % Here is where we can return...
           colliding = false; 
           return;
       end
       
       % Determine separating axis
       intervalDistance = abs(intervalDistance); 
       if intervalDistance < minIntervalDistance
           minIntervalDistance = intervalDistance; 
           translationAxis = axis;
           minEdgeIndex = edgeIndex; 
           
           d = polygonA.pos - polygonB.pos;
           if dot(d, translationAxis) > 0
              translationAxis = -translationAxis; 
           end
       end
    end
    
    % Create contact
    %p1 = ; 
    n = translationAxis;
    psi_n = -minIntervalDistance;
    % Determine p1
    overlap_poly = sutherlandHodgman( polygonA.verts_world, polygonB.verts_world );
    p1 = mean(overlap_poly,1);
    
    C = Contact(polygonA.bodyID, polygonB.bodyID,p1,n,psi_n);
    

end












