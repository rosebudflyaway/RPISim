

%% TODO: THIS FUNCTION IS OBSOLETE 

function C = cd_collide_poly_poly( A, B )

    %% Collide verts of A onto B
    C = [ cd_vertex_poly(A,B) cd_vertex_poly(B,A) ];

end

