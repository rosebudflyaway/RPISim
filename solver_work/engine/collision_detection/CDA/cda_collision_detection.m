
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
% cda_collision_detection.m 
%
% Performs collision detection on all bodies P.
% Returns a set of contacts C for polygonal bodies,
% for collision detection aware time stepper


function cda_collision_detection( obj )

    % FIXME: don't clear the active set of contacts every time
    % Clear sets of contacts
    obj.Contacts = [];  % Vector of contacts
    obj.activeBodies = [];
    cID = 0; % Contact ID
    bID = 0; % Body ID
    
    num_bodies = length(obj.P); % number of bodies 
    %ConsPerBody = zeros(num_bodies, 1); % Initialize sums of contacts for each body
    obj.num_subContacts = 0;

    % Maybe temporary, but necessary for now:
    for i=1:length(obj.P)
        if strcmp(obj.P{i,1}.body_type, 'mesh')
            obj.P{i,1}.update_world_position;  % Redundantly does static objects
        end
    end
    
    %  Compare each body to every other body
    for i = 1:num_bodies-1
      B1 = obj.P{i,1};
      for j = i+1:num_bodies
        B2 = obj.P{j,1};
        % Check bounding sphere.  Also, don't do CD on two static bodies
        if (B1.static==1 && B2.static==1) || norm(B2.u-B1.u) > B1.bound+B2.bound %(norm(B2.u-B1.u) > max(1,.4*max(norm(B1.nu(1:3)),norm(B2.nu(1:3))))*(B1.bound + B2.bound))
            continue;
        end
        
        %% POLY-POLY
        if strcmp(B1.body_type,'mesh') && strcmp(B2.body_type,'mesh')
            % Collide B1 into B2   
            if ~(B1.static==1 && B2.static==1)
                [C_j ns_j] = cda_collide_poly_poly(B1, i, B2, j, cID);  
                if ~isempty(C_j)
                    obj.Contacts = [obj.Contacts; C_j];
                    obj.num_subContacts = obj.num_subContacts + ns_j;
                    if B1.static == 0 
                        B1.ContactCount = B1.ContactCount + length(C_j); 
                        if B1.BodyIndex < 1
                           bID = bID+1;
                           B1.BodyIndex = bID;  
                           obj.activeBodies = [obj.activeBodies; i];
                        end
                    end
                    if B2.static == 0 
                        B2.ContactCount = B2.ContactCount + length(C_j);
                        if B2.BodyIndex < 1
                           bID = bID+1;
                           B2.BodyIndex = bID;  
                           obj.activeBodies = [obj.activeBodies; j];
                        end
                    end
                    cID = cID + length(C_j);
                end
            % Collide B2 into B1
                [C_j ns_j] = cda_collide_poly_poly(B2, j, B1, i, cID);  
                if ~isempty(C_j)
                    obj.Contacts = [obj.Contacts; C_j];
                    obj.num_subContacts = obj.num_subContacts + ns_j;
                    if B1.static == 0 
                        B1.ContactCount = B1.ContactCount + length(C_j); 
                        if B1.BodyIndex < 1
                           bID = bID+1;
                           B1.BodyIndex = bID;  
                           obj.activeBodies = [obj.activeBodies; i];
                        end
                    end
                    if B2.static == 0 
                        B2.ContactCount = B2.ContactCount + length(C_j);
                        if B2.BodyIndex < 1
                           bID = bID+1;
                           B2.BodyIndex = bID;  
                           obj.activeBodies = [obj.activeBodies; j];
                        end
                    end
                    cID = cID + length(C_j);
                end
            end
            
        %% POLY-SPHERE (Note: poly-sphere != sphere-poly)
        elseif strcmp(B1.body_type,'mesh') && strcmp(B2.body_type,'sphere')
            has_valid_contact = false; 
            ms_psi_n = inf;
            ms_norm = zeros(3,1); 
            ms_p1 = zeros(3,1); 
            
            for f=1:B1.num_faces    % For each triangle face
                
                % Put triangle together 
                f_j = B1.faces(f);
                Tri = [ B1.verts(f_j.verts(1)).world_coords ...
                        B1.verts(f_j.verts(2)).world_coords ...
                        B1.verts(f_j.verts(3)).world_coords ]' ;

                [triDist p1] = pointTriangleDistance(Tri,B2.u');
                n = (B2.u - p1'); 
                n = n/norm(n); 
                psi_n = triDist - B2.radius; 
                
                if psi_n > -.1 && abs(psi_n) < abs(ms_psi_n) && psi_n < 0.3     % TODO: fix epsilons hardcoded in
                    has_valid_contact = true;
                    ms_psi_n = psi_n;
                    ms_norm = n;
                    ms_p1 = p1';  
                end

            end
            
            if has_valid_contact
                cID = cID + 1;     % spheres only make a single sub-contact 
                obj.Contacts = [obj.Contacts; contact(cID, i, j, ms_norm, arbitraryTangent(n), ms_p1-B1.u, -B2.radius*ms_norm, ms_psi_n)];
                obj.num_subContacts = obj.num_subContacts + 1;
                if B1.static == 0 
                        B1.ContactCount = B1.ContactCount + 1; 
                        if B1.BodyIndex < 1
                           bID = bID+1;
                           B1.BodyIndex = bID;  
                           obj.activeBodies = [obj.activeBodies; i];
                        end
                end
                if B2.static == 0 
                    B2.ContactCount = B2.ContactCount + 1;
                    if B2.BodyIndex < 1
                       bID = bID+1;
                       B2.BodyIndex = bID;  
                       obj.activeBodies = [obj.activeBodies; j];
                    end
                end
            end
            
            
        elseif strcmp(B1.body_type,'mesh') && strcmp(B2.body_type,'union')
            for k=1:B2.num      % for every sphere in the union
                has_valid_contact = false; 
                ms_psi_n = inf;
                ms_norm = zeros(3,1); 
                ms_p1 = zeros(3,1); 
                ms_p2 = zeros(3,1); 
                for f=1:B1.num_faces    % for every face
                    
                    % Put triangle together 
                    f_j = B1.faces(f);
                    Tri = [ B1.verts(f_j.verts(1)).world_coords ...
                            B1.verts(f_j.verts(2)).world_coords ...
                            B1.verts(f_j.verts(3)).world_coords ]' ;

                    [triDist p1] = pointTriangleDistance(Tri,B2.UNION(k).u');
                    n = (B2.UNION(k).u - p1'); 
                    n = n/norm(n); 
                    psi_n = triDist - B2.UNION(k).radius; 

                    if psi_n > -.1 && abs(psi_n) < abs(ms_psi_n) && psi_n < 0.3     % Epsilons hardcoded in
                        has_valid_contact = true;
                        ms_psi_n = psi_n;
                        ms_norm = n;
                        ms_p1 = p1';  
                        ms_p2 = B2.UNION(k).offset - n*B2.UNION(k).radius;          % MINUS B2.u ??????????????????????????
                    end
                end
                if has_valid_contact
                    cID = cID + 1;     % spheres only make a single sub-contact 
                    obj.Contacts = [obj.Contacts; contact(cID, i, j, ms_norm, arbitraryTangent(n), ms_p1-B1.u, ms_p2, ms_psi_n)];
                    obj.num_subContacts = obj.num_subContacts + 1;
                    if B1.static == 0 
                            B1.ContactCount = B1.ContactCount + 1; 
                            if B1.BodyIndex < 1
                               bID = bID+1;
                               B1.BodyIndex = bID;  
                               obj.activeBodies = [obj.activeBodies; i];
                            end
                    end
                    if B2.static == 0 
                        B2.ContactCount = B2.ContactCount + 1;
                        if B2.BodyIndex < 1
                           bID = bID+1;
                           B2.BodyIndex = bID;  
                           obj.activeBodies = [obj.activeBodies; j];
                        end
                    end
                end
            end
            
            
            elseif strcmp(bi.body_type,'plane') && strcmp(bj.body_type,'cylinder')
            n = bi.n;                 % Normal from plane to cylinder
            t = arbitraryTangent(n);  % Tangent of plane 
            zworld = quatrotate(bj.quat,[0 0 1])';  % Cylinder's "up" vector
            %r = bj.radius*arbitraryTangent(zworld); % Vector perp to cylinder, length of its radius
            h = (bj.height/2)*zworld; % Vector parallel to cylinder, half the length of its height
            
            d = dot(n,zworld); 
            if d == 1 || d==-1
                r = bj.radius*arbitraryTangent(zworld); % Vector perp to cylinder, length of its radius
            else
                c2p = quatrotate(quatinv(bj.quat), -n');     % Negative normal in cylinder's frame 
                c2p = [c2p(1:2) 0];
                c2p = c2p/norm(c2p); 
                r = bj.radius*quatrotate(bj.quat, c2p)'; % Vector perp to cylinder, length of its radius
            end
            
            if d>0.8        % Bottom of cylinder is facing down
                p2 = bj.u + r - h;  
                psi_n = dot(n,p2-bi.u);
                p1 = p2-psi_n*n;  
                obj.Contacts = [obj.Contacts; contact(cid,i,j,n,t, p1-bi.u, p2-bj.u, psi_n)];
                cid=cid+1; 
                
                p2 = bj.u + rot(zworld,(2/3)*pi) * r - h;
                psi_n = dot(n,p2-bi.u);
                p1 = p2-psi_n*n;  
                obj.Contacts = [obj.Contacts; contact(cid,i,j,n,t, p1-bi.u, p2-bj.u, psi_n)];
                cid=cid+1; 
                
                p2 = bj.u + rot(zworld,-(2/3)*pi) * r - h;
                psi_n = dot(n,p2-bi.u);
                p1 = p2-psi_n*n;  
                obj.Contacts = [obj.Contacts; contact(cid,i,j,n,t, p1-bi.u, p2-bj.u, psi_n)];
                cid=cid+1; 
                
            elseif d<-0.8    % Top of cylinder is facing down
                p2 = bj.u + r + h;  
                psi_n = dot(n,p2-bi.u);
                p1 = p2-psi_n*n;  
                obj.Contacts = [obj.Contacts; contact(cid,i,j,n,t, p1-bi.u, p2-bj.u, psi_n)];
                cid=cid+1; 
                
                p2 = bj.u + rot(zworld,(2/3)*pi) * r + h;
                psi_n = dot(n,p2-bi.u);
                p1 = p2-psi_n*n;  
                obj.Contacts = [obj.Contacts; contact(cid,i,j,n,t, p1-bi.u, p2-bj.u, psi_n)];
                cid=cid+1; 
                
                p2 = bj.u + rot(zworld,-(2/3)*pi) * r + h;
                psi_n = dot(n,p2-bi.u);
                p1 = p2-psi_n*n;  
                obj.Contacts = [obj.Contacts; contact(cid,i,j,n,t, p1-bi.u, p2-bj.u, psi_n)];
                cid=cid+1; 
                
            else            % Cylinder is mostly on its side
                p2 = bj.u + r - h;  
                psi_n = dot(n,p2-bi.u);
                p1 = p2-psi_n*n;  
                obj.Contacts = [obj.Contacts; contact(cid,i,j,n,t, p1-bi.u, p2-bj.u, psi_n)];
                cid=cid+1; 
                
                p2 = bj.u + r + h;  
                psi_n = dot(n,p2-bi.u);
                p1 = p2-psi_n*n;  
                obj.Contacts = [obj.Contacts; contact(cid,i,j,n,t, p1-bi.u, p2-bj.u, psi_n)];
                cid=cid+1; 
            end
            
            % We are always adding contacts for now
            if bi.static == 0 
                bi.ContactCount = bi.ContactCount + 1; 
                if bi.BodyIndex < 1
                   bID = bID+1;
                   bi.BodyIndex = bID;  
                   obj.activeBodies = [obj.activeBodies; i];
                end
            end
            if bj.static == 0 
                bj.ContactCount = bj.ContactCount + 1;
                if bj.BodyIndex < 1
                   bID = bID+1;
                   bj.BodyIndex = bID;  
                   obj.activeBodies = [obj.activeBodies; j];
                end
            end
            
            
        end

      end % i
    end % j
     
end










