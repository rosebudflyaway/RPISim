
% Given a body B, scales the body properties, particularly the vertex data.
function B = body_scale( B, scl )

    % Scale vertices
    for v=1:B.num_verts
       B.verts_local(v,:) = scl*B.verts_local(v,:);  
    end
    
    % Scale mass
    B.mass = scl*B.mass;
    B.J = scl*B.J;

end

