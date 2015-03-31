

% This function updates mesh-specific attributes of a body B

function B = body_updateMesh( B )

    % Update world coordinates
    for v = 1:B.num_verts
        B.verts_world(v,:) = (B.pos + [cos(B.rot) -sin(B.rot); sin(B.rot) cos(B.rot)]*B.verts_local(v,:)')';
    end

    % Update bounding box 
    maxBBexpand = 0.4;      % TODO: replace hard-coded epsilons
    
    AABB_min = min(B.verts_world);
    AABB_max = max(B.verts_world); 
    vec = AABB_max-AABB_min;
    length = norm(vec);
    expansion = min(maxBBexpand, 0.5*length);   % Increase bounding box by 30% or maxBBexpand along diagonal.
    B.AABB_min = AABB_min - expansion*vec;     
    B.AABB_max = AABB_max + expansion*vec;    
    
end

