


function [colliding collision_contact] = cd_poly_poly_with_correction( bodyA, bodyB )

    colliding = false; 
    collision_contact = [];
    
    A = bodyA.verts_world; 
    B = bodyB.verts_world; 
    b1id = bodyA.bodyID;
    b2id = bodyB.bodyID; 
    
%     % Determine overlap polygon
%     overlap_poly = sutherlandHodgman( A, B );
%     if isempty(overlap_poly)
%         return; 
%     else        
%         colliding = true; 
%         % Determine centroid of overlap
%         p1 = mean(overlap_poly,1);
% % %         [ebB nB psiB] = nearestEdgeDistance( p1, bodyB );
% % %         [ebA nA psiA] = nearestEdgeDistance( p1, bodyA );
% % %         
% % %         if psiA < psiB
% % %             n = nA;
% % %             VA = A(ebA,:);
% % %             psi = 0;
% % %             % Find deepest penetration of bodyB against ebA
% % %             for vb=1:bodyB.num_verts
% % %                 temp_psi = dot(VA-B(vb,:),n);
% % %                 if temp_psi < psi
% % %                    psi = temp_psi;
% % %                 end
% % %             end
% % %         else
% % %             n = nB;
% % %             VB = B(ebB,:);
% % %             psi = 0;
% % %             % Find deepest penetration of bodyB against ebA
% % %             for va=1:bodyA.num_verts
% % %                 temp_psi = dot(VB-A(va,:),n);
% % %                 if temp_psi < psi
% % %                    psi = temp_psi;
% % %                 end
% % %             end
% % %         end
%         
%          
%         r1 = p1-bodyA.pos';
%         r2 = p1-bodyB.pos';
%         r3 = r1+r2;
%         n = [-r3(2) r3(1)]; 
%         n = n/norm(n);
%         if dot(bodyA.pos'-p1,n) > 0
%             n = -n;
%         end
%         psi = -10*polyarea(overlap_poly(:,1), overlap_poly(:,2));
%          
%         
%         collision_contact = Contact(bodyA.bodyID, bodyB.bodyID,p1,n,psi);
%         
%     end
    
    
    
    
    % Generate contacts for each vertex that is INSIDE the other body
    %C = [];
    A_p = [];
    A_n = [];
    A_psi = [];
    
    C = []; 
    
    % A's vertices
    VAinB = inpolygon(A(:,1),A(:,2),B(:,1),B(:,2)); 
    if any(VAinB)
        colliding = true; 
        for va=1:bodyA.num_verts
           if VAinB(va)
               % Find nearest edge of B to va
               nb = B(1,:)-B(end,:);
               nb = nb/norm(nb);
               nb = [nb(2) -nb(1)]; 
               psi_b = dot(A(va,:)-B(1,:),nb);
               MIN_PSI_ab = psi_b;
               MIN_NB = nb; 
               MIN_P1 = A(va,:);
               for vb=2:bodyB.num_verts
                   nb = B(vb,:)-B(vb-1,:);
                   nb = nb/norm(nb);
                   nb = [nb(2) -nb(1)]; 
                   psi_b = dot(A(va,:)-B(vb,:),nb);
                   if psi_b > MIN_PSI_ab
                       MIN_PSI_ab = psi_b;
                       MIN_NB = nb; 
                       MIN_P1 = A(va,:); 
                   end
               end
               A_p = [A_p; MIN_P1];
               A_n = [A_n; MIN_NB];  
               A_psi = [A_psi; MIN_PSI_ab]; 
           end
        end
         % Average A's contacts
        C = Contact(bodyA.bodyID, bodyB.bodyID, mean(A_p,1), -mean(A_n,1), mean(A_psi));  
    end

    B_p = [];
    B_n = [];
    B_psi = [];  
    % B's vertices
    VBinA = inpolygon(B(:,1),B(:,2),A(:,1),A(:,2)); 
    if any(VBinA)
        colliding = true; 
        for vb=1:bodyB.num_verts
           if VBinA(vb)
               % Find nearest edge of A to vb
               na = A(1,:)-A(end,:);
               na = na/norm(na);
               na = [na(2) -na(1)]; 
               psi_a = dot(B(vb,:)-A(1,:),na);
               MIN_PSI_ba = psi_a;
               MIN_NA = na; 
               MIN_P1 = B(vb,:);
               for va=2:bodyA.num_verts
                   na = A(va,:)-A(va-1,:);
                   na = na/norm(na);
                   na = [na(2) -na(1)]; 
                   psi_a = dot(B(vb,:)-A(va,:),na);
                   if psi_a > MIN_PSI_ba
                       MIN_PSI_ba = psi_a;
                       MIN_NA = na; 
                       MIN_P1 = B(vb,:); 
                   end
               end
               B_p = [B_p; MIN_P1];
               B_n = [B_n; MIN_NA];  
               B_psi = [B_psi; MIN_PSI_ba]; 
           end
        end
        % Average B's contacts
        C = [C Contact(bodyB.bodyID, bodyA.bodyID, mean(B_p,1), -mean(B_n,1), mean(B_psi))];  
    end
    
    collision_contact = C;

end















