


function sim = collision_detection( sim )

    % TODO: remove hard-coded epsilons
    %eps_ve = 10.0*sim.h;  % Vertex-edge epsilon
    
    %eps_ve = max( [(0.5*sim.h)  (0.8*.001)]); 
    eps_ve = 0.07; 

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
            
            % - Don't collide static bodies
            % - Some bodies are listed as "doNotCollide" with each other,
            % e.g. bodies with joints that require overlap.
            % - Broad-phase test
            if ~A.dynamic && ~B.dynamic || ...
               any(A.doesNotCollideWith == B.bodyID) || ...
               ~cd_intersect_AABBs(A,B)
                continue; 
            end   
            
            cab = cd_vertex_poly(A,B, eps_ve);
            cba = cd_vertex_poly(B,A, eps_ve);
            C = [C cab cba];
            
            if ~isempty(cab) || ~isempty(cba)
               sim = sim_activateBodies(sim, A.bodyID, B.bodyID);  
            end
            
        end
    end
    
    sim.contacts = C; 
end

