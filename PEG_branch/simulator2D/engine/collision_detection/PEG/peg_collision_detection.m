

% Given two bodies B1 and B2, performs collision detection 
% Returns an array of contacts C as well as a set of inter-contact
% constraints X.  
function sim = peg_collision_detection( sim )

    % TODO: remove hard-coded epsilons
    eps_ve = 0.05;  % Vertex-edge epsilon

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
            
            cab = peg_vertex_poly(A,B, eps_ve);
            cba = peg_vertex_poly(B,A, eps_ve);
            C = [C cab cba];
            
            if ~isempty(cab) || ~isempty(cba)
               sim = sim_activateBodies(sim, A.bodyID, B.bodyID);  
            end
            
        end
    end
    
    sim.contacts = C; 

end

