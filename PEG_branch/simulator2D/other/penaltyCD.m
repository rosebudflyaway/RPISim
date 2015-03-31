

function sim = penaltyCD( sim )

    %% Clear previous contacts
    C = [];
    [sim.bodies.active] = deal(false);  
    sim.num_activeBodies = 0; 
    
    for b1id = 1:sim.num_bodies-1
       for b2id = b1id+1:sim.num_bodies
          A = sim.bodies(b1id);
          B = sim.bodies(b2id);
          if ~A.dynamic && ~B.dynamic || any(A.doesNotCollideWith == B.bodyID) || ~cd_intersect_AABBs(A,B)
                continue; 
          end   
          
          [colliding collision_contact] = PolygonCollision( A, B );
          
          if colliding
             C = [C collision_contact]; 
             sim = sim_activateBodies(sim, A.bodyID, B.bodyID);  
          end
          
           
       end
    end
    sim.contacts = C;

end

