



function sim = PEG_collision_detection( sim )

    % TODO: remove hard-coded epsilons
    eps_ve = 0.25;  % Vertex-edge epsilon

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
            
            %% Mid-phase 
            % Determine feature sets
            %[ vb_min, vb_max ] = getActiveEdges2d( A, B, eps_ve ); 
            %[ va_min, va_max ] = getActiveEdges2d( B, A, eps_ve ); 
            
            Cab = PEG_middle_phase(A,B,eps_ve);
            Cba = PEG_middle_phase(B,A,eps_ve);
            
            sim.contacts = [C Cab Cba];
            sim_drawContacts(sim); 
            
            
%             if ~isempty(va_min) 
%                 %% Narrow-phase
%                 [c, u, i, x] = PEG_narrow_phase(A, va_min, va_max, B, vb_min, vb_max, eps_ve); 
%             end
            
       end
    end
    
    

end

