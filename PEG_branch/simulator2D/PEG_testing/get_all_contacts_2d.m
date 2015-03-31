


% Given a Simulation object sim, finds all potential vertex-edge contacts
% between every pair of bodies.  

function sim = get_all_contacts_2d( sim  )


    %eps_ve = 0.025; 
    %eps_ve = max(6*sim.h,.015); 
    %eps_ve = eps_ve*eps_ve;  
    %eps_small = 0.015^2;  
    %eps_small = 0.05^2; 
    
%     
%     if sim.h >= 0.025
%         eps_small = 0.05^2;
%     elseif sim.h >= 0.016
%         eps_small = 0.025^2; 
%     elseif sim.h >= 0.011
%         eps_small = 0.023^2; 
%     elseif sim.h >= 0.008
%         eps_small = 0.02^2;
%     else
%         eps_small = 0.015^2; 
%     end
    
    %eps_small = (2*sim.h)^2; 
    
    eps_small = max( [(0.5*sim.h)^2  (0.8*.001)^2]); 

    function C = vertex_edge_2d(A, B)
        C = []; 
        
        % Project forward by h
        A_proj = A; 
        A_proj.pos = A_proj.pos + sim.h*A_proj.nu(1:2); 
        A_proj.rot = A_proj.rot + sim.h*A_proj.nu(3); 
        A_proj = body_updateMesh( A_proj );
        B_proj = B; 
        B_proj.pos = B_proj.pos + sim.h*B_proj.nu(1:2); 
        B_proj.rot = B_proj.rot + sim.h*B_proj.nu(3); 
        B_proj = body_updateMesh( B_proj );
        
        for va_iter = 1:A.num_verts
            va = A.verts_world(va_iter,:);  
            va_proj = A_proj.verts_world(va_iter,:);  
            for eb_iter = 1:B.num_verts-1
                % Determine squared distance of current position
                vb1 = B.verts_world(eb_iter,:); 
                vb2 = B.verts_world(eb_iter+1,:); 
                sqD = sqDist_point_segment2d( vb1, vb2, va ); 
                
                % Determine squared distance of projected position
                vb1_proj = B_proj.verts_world(eb_iter,:); 
                vb2_proj = B_proj.verts_world(eb_iter+1,:); 
                sqD_proj = sqDist_point_segment2d( vb1_proj, vb2_proj, va_proj ); 
                
                if sqD <= eps_small || sqD_proj <= eps_small  % sqD <= eps_ve
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
            % Determine squared distance of current position
            vb1 = B.verts_world(eb_iter,:); 
            vb2 = B.verts_world(1,:); 
            sqD = sqDist_point_segment2d( vb1, vb2, va ); 
            
            % Determine squared distance of projected position
            vb1_proj = B_proj.verts_world(eb_iter,:); 
            vb2_proj = B_proj.verts_world(1,:); 
            sqD_proj = sqDist_point_segment2d( vb1_proj, vb2_proj, va_proj ); 
            
            if sqD <= eps_small || sqD_proj <= eps_small % sqD <= eps_ve
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
    
      % TODO: This is already done in preDynamics()
      nj = length(sim.joints); 
      for j=1:nj
         sim = sim_activateBodies( sim, sim.joints(j).body1id, sim.joints(j).body2id );
      end
      njc = sim.num_jointConstraints; 

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
            if ~A.dynamic && ~B.dynamic || any(A.doesNotCollideWith == B.bodyID) || ~cd_intersect_AABBs(A,B)
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







