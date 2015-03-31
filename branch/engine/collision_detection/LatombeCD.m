

% Performs collision detection using rigid body constraints as described in 
% "Robot Motion Planning" by Jean-Claude Latombe

function LatombeCD( sim, B1,B2 )

b1id = B1.bodyID;
b2id = B2.bodyID; 

sim.Contacts = [];  % TODO: just debugging here


%% Constraint A (vertex-face)
% TODO



%% Constraint C (edge-edge)
if b1id < b2id  % Avoid redundant edge-edge contacts.  
    
    % for every edge in B1, B2
    for incr_eA = 1:B1.num_edges
      for incr_eB = 1:B2.num_edges
          
          % Calculate unsigned gap distance and nearest points on edges
          [d, ep1, ep2] = cda_calc_distance_edge_edge(B1, B2, incr_eA, incr_eB);
          
          if d > .1         % TODO: a hard coded epsilon
              continue;
          end
          
          % The edges
          E1 = B1.edges(incr_eA);
          E2 = B2.edges(incr_eB); 
          
          % Vectors (un-normalized) representing the edges
          e1 = B1.verts(E1.verts(2)).world_coords - B1.verts(E1.verts(1)).world_coords;
          e2 = B2.verts(E2.verts(2)).world_coords - B2.verts(E2.verts(1)).world_coords;
          
          % Calculate the normal of the edge-edge contact
          eX = cross3(e1,e2);       
            if norm(eX) == 0
                continue;        % Don't do parallel edges 
            end
          n = sign( dot( eX, E2.t1+E2.t2) ) * eX ;
          n = n/norm(n)
          
          % Determine applicability APPL of edge-edge constraint
          tA1 = qtrotate(B1.quat, E1.t1);
          tA2 = qtrotate(B1.quat, E1.t2);
          tB1 = qtrotate(B2.quat, E2.t1);
          tB2 = qtrotate(B2.quat, E2.t2);
          
          disp('t dot products:');
          ka1 = sign( dot(tA1, n) );    dot(tA1, n)
          ka2 = sign( dot(tA2, n) );    dot(tA2, n)
          kb1 = sign( dot(tB1, n) );    dot(tB1, n)
          kb2 = sign( dot(tB2, n) );    dot(tB2, n)
            if ka1==0 || ka2==0 || kb1==0 || kb2==0 
                continue; 
            end
            
          if ka1<0 && ka2<0 && kb1<0 && kb2<0 
             disp(['Positive: ' num2str(incr_eA) ' ' num2str(incr_eB)]); 
          end
          
          % The applicability does NOT determine penetration.  It only
          % decides if the edge-edge contact is feasible i.e. there is
          % the possibility of a dividing plane at the E-E contact.  
          APPL = (ka1==ka2 && kb1==kb2 && ka1~=kb1)
          
          a = B1.verts(E1.verts(1)).world_coords;
          b = B2.verts(E2.verts(1)).world_coords;  
          fq = dot( n, b-a )    %: f(q) determines penetration WHEN there is applicability
          
          %psi = d;
          psi = fq;
          
          sim.addContact(b1id,b2id,n,arbitraryTangent(n), ep1-B1.u, ep2-B2.u, psi);
          
      end
    end

end



