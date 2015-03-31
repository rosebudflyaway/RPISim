


% Given a Simulation object sim, finds all potential vertex-edge contacts
% between every pair of bodies.  

function sim = get_all_contacts_2d( sim  )

    eps_ve = 0.3; 
    eps_ve = eps_ve*eps_ve;  

    function C = vertex_edge_2d(A, B)
        C = []; 
        for va_iter = 1:A.num_verts
            va = A.verts_world(va_iter,:);  
            for eb_iter = 1:B.num_verts-1
                % Determine squared distance 
                vb1 = B.verts_world(eb_iter,:); 
                vb2 = B.verts_world(eb_iter+1,:); 
                sqD = sqDist_point_segment2d( vb1, vb2, va ); 
                if sqD <= eps_ve
                    n = edge_normal(B,eb_iter);  
                    psi = dot( n, vb1-va);
                    c = Contact(A.bodyID, B.bodyID, va, n, psi);  
                    c.f1id = va_iter;
                    c.f2id = eb_iter;  
                    c.applicability = APPL_vertex_edge(A,va_iter,B,eb_iter); 
                    C = [C c]; 
                end
            end
            eb_iter = B.num_verts;  
            % Determine squared distance 
            vb1 = B.verts_world(eb_iter,:); 
            vb2 = B.verts_world(1,:); 
            sqD = sqDist_point_segment2d( vb1, vb2, va ); 
            if sqD <= eps_ve
                n = edge_normal(B,eb_iter);  
                psi = dot( n, vb1-va);
                c = Contact(A.bodyID, B.bodyID, va, n, psi);  
                c.f1id = va_iter;
                c.f2id = eb_iter;  
                c.applicability = APPL_vertex_edge(A,va_iter,B,eb_iter); 
                C = [C c]; 
            end
        end
    end

    %% Clear previous contacts
    C = [];
    [sim.bodies.active] = deal(false);  
    sim.num_activeBodies = 0; 

    %% Iterate over body pairs 
    num_bodies = length(sim.bodies);
    for Aid = 1:num_bodies-1 
        A = sim.bodies(Aid);
        for Bid = Aid+1:num_bodies
            B = sim.bodies(Bid); 
            
            % Don't collide static bodies
            % Also, some bodies are listed as "doNotCollide" with each other,
            %        e.g. bodies with joints that require overlap.
            %% Broad-phase test
            if ~A.dynamic && ~B.dynamic || ...
               any(A.doesNotCollideWith == B.bodyID) || ...
               ~cd_intersect_AABBs(A,B)
                continue; 
            end   
            
            Ca = vertex_edge_2d(A,B);
            Cb = vertex_edge_2d(B,A); 
            if ~isempty(Ca) || ~isempty(Cb)
                C = [ C Ca Cb ];
                sim = sim_activateBodies( sim, Aid, Bid ); 
            end 
       end
    end
    
    sim.contacts = C; 
    
end

