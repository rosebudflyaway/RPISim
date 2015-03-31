

function sim = penalty_cd( sim )

    eps_vf = 10*sim.h; %.1; 
    eps_ee = 10*sim.h; %.1; 

    %% Clear previous contacts
    C = [];
    [sim.bodies.active] = deal(false);  
    sim.num_activeBodies = 0; 
    
    function c = pointFaceContact(bdy,fc)
       fb = bdy.faces(fc,:); 
       n = bdy.face_norms(fc,:);  
       psi = dot3( n, P - bdy.verts_world(fb(1),:));
       if bdy.bodyID == A.bodyID
          c = Contact(A.bodyID, B.bodyID, P, n, psi);
       else
          c = Contact(A.bodyID, B.bodyID, P, -n, psi);
       end
       c.f1id = 0;
       c.f2id = fc;
       c.type = 'vf';  % 1 => vertex-face
    end
    

    %% Iterate over body pairs 
    num_bodies = length(sim.bodies);
    for Aid = 1:num_bodies-1 
        A = sim.bodies(Aid);
        for Bid = Aid+1:num_bodies
            B = sim.bodies(Bid); 

            % Broad-phase test
            if ~A.dynamic && ~B.dynamic || any(A.doesNotCollideWith == B.bodyID) || ~cd_intersect_AABBs(A,B)
                continue; 
            end   
            
            % Determine volume of overlap and contact per body pair
            [V,P] = volume_of_overlap(A,B);
            if V > 0
               sim = sim_activateBodies( sim, Aid, Bid );
               % Find nearest face of A to P
               ca_best = pointFaceContact(A,1); 
               for fa_iter = 2:A.num_faces
                  ca_temp = pointFaceContact(A,fa_iter); 
                  if ca_temp.psi_n > ca_best.psi_n
                     ca_best = ca_temp; 
                  end
               end
               
               % Find nearst face of B to P
               cb_best = pointFaceContact(B,1); 
               for fb_iter = 2:B.num_faces
                  cb_temp = pointFaceContact(B,fb_iter); 
                  if cb_temp.psi_n > cb_best.psi_n
                     cb_best = cb_temp; 
                  end
               end
            
               ca_best.psi_n = 2*ca_best.psi_n;
               cb_best.psi_n = 2*cb_best.psi_n;
               
               if ca_best.psi_n > cb_best.psi_n
                  C = [C ca_best];
               else
                  C = [C cb_best]; 
               end
               
            end
            
       end
    end
    
    sim.contacts = C; 


end

