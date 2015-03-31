

function [ min, max ] = ProjectPolygon( axis, polygon )

    dotProduct = dot(axis, polygon.verts_world(1,:)');
    min = dotProduct;
    max = dotProduct; 
    for i = 1 : polygon.num_verts
       dotProduct = dot(axis, polygon.verts_world(i,:)'); 
       if dotProduct < min
           min = dotProduct;
       elseif dotProduct > max
           max = dotProduct;
       end
    end

end

