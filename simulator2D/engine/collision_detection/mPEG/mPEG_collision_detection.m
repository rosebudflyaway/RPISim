


function sim = mPEG_collision_detection( sim )

% TODO: remove hard-coded epsilons
eps_ve = 1.2;  % Vertex-edge epsilon

%% Clear previous contacts
C = [];
[sim.bodies.active] = deal(false);  
sim.num_activeBodies = 0; 
sim.contacts = [];
sim.Iconstraints = [];
sim.Xconstraints = []; 

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
    
    c = []; 
    ic = [];
    xc = []; 

    %% In 2D, there are two cases for each vertex
    %  (i)  Vertex-edge
    %  (ii) Vertex-edges (vertex-vertex)
    
    [Dab, vai, vbi] = getNearestVerts(A,B);
    va = A.verts_world(vai,:);
    vb = B.verts_world(vbi,:);
    
    
    %% At this point, va and vb are the two nearest vertices
    % TODO: don't assume the only contact is C(va,vb) 
    if Dab < eps_ve^2 
        % Create the 4 contacts, i.e. both vertices against the other body's edges
        % va on Eb1 and Eb2
        Eb1 = getEdge(B,vbi-1);
        Eb2 = getEdge(B,vbi); 
        nb1 = [Eb1(2) -Eb1(1)];  nb1 = nb1/norm(nb1);
        nb2 = [Eb2(2) -Eb2(1)];  nb2 = nb2/norm(nb2);
        psi_b1 = dot( nb1, va-vb );
        psi_b2 = dot( nb2, va-vb );
        c = [c Contact( A.bodyID, B.bodyID, va, -nb1, psi_b1 )]; % c1
        c = [c Contact( A.bodyID, B.bodyID, va, -nb2, psi_b2 )]; % c2
        % vb on Ea1 and Ea2
        Ea1 = getEdge(A,vai-1);
        Ea2 = getEdge(A,vai); 
        na1 = [Ea1(2) -Ea1(1)];  na1 = na1/norm(na1);
        na2 = [Ea2(2) -Ea2(1)];  na2 = na2/norm(na2);
        psi_a1 = dot( na1, vb-va );
        psi_a2 = dot( na2, vb-va );
        c = [c Contact( B.bodyID, A.bodyID, vb, -na1, psi_a1 )]; % c3
        c = [c Contact( B.bodyID, A.bodyID, vb, -na2, psi_a2 )]; % c4
        
        % Create I-constraints
        ic = [ic Iconstraint(1,2)];
        ic = [ic Iconstraint(3,4)];
        
        % Create X-constraints
        xc = [xc Xconstraint(1,4)];
        xc = [xc Xconstraint(2,3)]; 
        
    end
    
    if ~isempty(c) 
      sim = sim_activateBodies(sim, A.bodyID, B.bodyID);  
      C = [C c]; 
      
      if ~isempty(ic), sim.Iconstraints = [sim.Iconstraints ic]; end
      if ~isempty(xc), sim.Xconstraints = [sim.Xconstraints xc]; end
    end

  end
end

sim.contacts = C; 

end


















