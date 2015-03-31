
% Given bodies A and B, returns a set of contacts 

function C = cd_vertex_poly( A, B, eps_ve )

    C = [];
    body1_id = A.bodyID;
    body2_id = B.bodyID;
    
    % For each vertex in A
    for v = 1:A.num_verts
       V = A.verts_world(v,:); 
       
       % For each edge in B
       for e=1:B.num_verts-1
           Vb1 = B.verts_world(e,:);
           Vb2 = B.verts_world(e+1,:);
           
           % Determine normal of edge by rotating 90 
           E = Vb2-Vb1;
           n = [E(2) -E(1)];  
           n = n/norm(n); 
           
           % Calculate distance to half space represented by edge
           psi_n = dot(n,V-Vb1);
           
           if psi_n < eps_ve && psi_n > -1*eps_ve
               %% Check if V is within the segment or not
               E = E/norm(E); 
               % Check if V is "far outside" of Vb1
               if dot(E,Vb1-V) > eps_ve 
                   continue; 
               % Check if V is "far outside" of Vb2
               elseif dot(E,V-Vb2) > eps_ve
                   continue;
               % Otherwise, V projects onto the segment
               else
                   C = [C Contact(body1_id, body2_id, V, -n, psi_n)];
               end
           end
       end
       % The last edge in B
       Vb1 = B.verts_world(B.num_verts,:);
       Vb2 = B.verts_world(1,:);
       
       % Determine normal of edge by rotating 90 
       E = Vb2-Vb1;
       n = [E(2) -E(1)];  
       n = n/norm(n); 

       % Calculate distance to half space represented by edge
       psi_n = dot(n,V-Vb1);

       if psi_n < eps_ve && psi_n > -1*eps_ve
           %% Check if V is within the segment or not
           E = E/norm(E); 
           % Check if V is "far outside" of Vb1
           if dot(E,Vb1-V) > eps_ve 
               continue; 
           % Check if V is "far outside" of Vb2
           elseif dot(E,V-Vb2) > eps_ve
                continue; 
           % Otherwise, V projects onto the segment
           else
               C = [C Contact(body1_id, body2_id, V, -n, psi_n)];
           end
       end
    end

end

