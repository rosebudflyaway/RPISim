
%%%%%%%%%% NOT USED


% This function updates sphere-specific attributes of a body B

function B = body_updateSphere( B )

    diagonal_vec = 1/sqrt(3) * [1 1 1];

    % Update bounding box 
    maxBBexpand = 0.05;      % TODO: replace hard-coded epsilons
    
    AABB_min = min(B.verts_world);
    AABB_max = max(B.verts_world); 
    vec = AABB_max-AABB_min;
    length = norm(vec);
    expansion = min(maxBBexpand, 0.15*length);   % Increase bounding box by 3% or maxBBexpand along diagonal.
    B.AABB_min = AABB_min - expansion*vec;     
    B.AABB_max = AABB_max + expansion*vec;    
    
end

