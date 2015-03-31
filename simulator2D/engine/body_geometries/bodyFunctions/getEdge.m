

% Given Body B and edge index ei, returns the vector E representing the edge.

function E = getEdge( B, ei )

    if ei < 1                       % Return last edge
        E = B.verts_world(1,:) - B.verts_world(B.num_verts,:);
    elseif ei >= B.num_verts        % Return first edge
        E = B.verts_world(1,:) - B.verts_world(B.num_verts,:);
    else                            % Return ei^th edge
        E = B.verts_world(ei+1,:) - B.verts_world(ei,:);
    end

end

