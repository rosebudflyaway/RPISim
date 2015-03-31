%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
% collision_detection.m 
%
% Performs collision detection on all bodies in obj.P
% Assigns a set of contacts C to sim.Contacts
% 
function collision_detection( sim ) 
 
% Maybe temporary, but necessary for now:
for i=1:length(sim.P)
    if strcmp(sim.P{i}.body_type, 'mesh')
        sim.P{i}.update_world_position;  % Redundantly does static objects
    end
end

num_bods = length(sim.P);  % Number of bodies
for i=1:num_bods-1
    bi = sim.P{i};
    for j=i+1:num_bods  % We're looking at bodies (bi bj) for i<j.  
        bj = sim.P{j};
        
        % Do not compare bodies that are set to not collide
        if bi.doesNotCollideWith(bj.bodyID)
            continue;
        end
        % Check bounding sphere.  Also, don't do CD on two static bodies
        %if (bi.static==1 && bj.static==1) || (norm(bj.u-bi.u) > max(1,.4*max(norm(bi.nu(1:3)),norm(bj.nu(1:3))))*(bi.bound + bj.bound))
        if (bi.static==1 && bj.static==1) || (norm(bj.u-bi.u) > 1.2*(bi.bound + bj.bound)) 
            continue;   % Note, more is done below for cyl-sph since we don't have bounding boxes implemented.
        end
        
        
        %% Cylinder - Particle
        if strcmp(bi.body_type,'cylinder') && strcmp(bj.body_type,'particle')
            n = [0;0;1];
            t = arbitraryTangent(n); 
            p1 = [bj.u(1); bj.u(2); 0] - bi.u;
            p2 = [0;0;0];
            psi_n = bj.u(3);
            sim.addContact(i,j,n,t,p1,p2,psi_n);
            
        elseif strcmp(bi.body_type,'mesh') && strcmp(bj.body_type,'mesh')
            collide_poly_poly(sim, i,j, bi,bj); 
        
        %% Sphere - Sphere
        elseif strcmp(bi.body_type,'sphere') && strcmp(bj.body_type,'sphere')
            n = bj.u-bi.u;                                                 % Normal direction
            n = n/norm(n);                                                 % Normalized
            t = arbitraryTangent(n);                                       % Choose a tangent.
            p1 = n*bi.radius;                                              % [x;y;z] Position of contact on body_1 in world frame
            p2 = -n*bj.radius;                                             % [x;y;z] Position of contact on body_2 in world frame
            psi_n = norm(bj.u-bi.u) - bj.radius - bi.radius;               % Penetration depth
            sim.addContact(i,j,n,t,p1,p2,psi_n);

            
        %% Sphere - Unions_of_spheres
        elseif strcmp(bi.body_type, 'sphere') && strcmp(bj.body_type, 'union')
            if(norm(bi.u-bj.u) > bi.bound + bj.bound + 0.25*bi.radius)
                continue;
            else
                for k=1:bj.num
                    %if(bi.static==0 && norm(bj.UNION(k).u - bi.u) < bi.bound + bj.UNION(k).bound + 1e-1)
                    %  union_cons = union_cons + 1;
                    %  normal of the two spheres
                    n = bj.UNION(k).u - bi.u;
                    n = n/norm(n);
                    t = arbitraryTangent(n);
                    p1 = n*bi.radius;    % r1_sphere in the Gn and Gf matrix
                    % r2(p2) is from the center of the union to the contact point
                    p2 = -n*bj.UNION(k).radius + bj.UNION(k).offset;  % r2_union in the Gn and Gf matrix
                    psi_n = norm(bj.UNION(k).u - bi.u) - bj.UNION(k).radius - bi.radius;
                    
                    if (psi_n < 1e-2)   % TODO: Fix hardcoded epsilons
                        sim.addContact(i, j, n, t, p1, p2, psi_n);
                    end
                end
            end
        %% Unions_of_spheres - Unions_of_spheres
        elseif strcmp(bi.body_type, 'union') && strcmp(bj.body_type, 'union')
            if(norm(bi.u-bj.u) > bi.bound + bj.bound + 0.15*bi.bound)
                continue;
            else
                for p = 1:bi.num
                    for q = 1:bj.num
                        n = bj.UNION(q).u - bi.UNION(p).u;
                        n = n/norm(n);
                        t = arbitraryTangent(n);
                        p1 = n*bi.UNION(p).radius + quatrotate(bi.quat,bi.UNION(p).offset')';  % the r vector in the union space
                        p2 = -n*bj.UNION(q).radius + quatrotate(bj.quat,bj.UNION(q).offset')'; % the r vector in the union space
                        psi_n = norm(bj.UNION(q).u - bi.UNION(p).u) - bj.UNION(q).radius - bi.UNION(p).radius;
                        if (psi_n < 1e-2)
                            sim.addContact(i, j, n, t, p1, p2, psi_n);
                        end
                    end
                end
            end
            
        %% Cylinder - Union 
        elseif strcmp(bi.body_type, 'cylinder') && strcmp(bj.body_type, 'union')
            % CASE 1 - union is closest to an end
            if(norm(bi.u-bj.u) > bi.bound + bj.bound + 0.15*bi.bound)
                continue;
            else
                for p = 1:bj.num
                    cz = (bi.height/2)*quatrotate(bi.quat,[0;0;1]')';      % z vector of cylinder, with length of half the cylinder's height
                    c2s = bj.UNION(p).u - bi.u;                            % Vector from center of cylinder to center of sphere, world frame
                    c2s = quatrotate(quatinv(bi.quat), c2s');              % Same, but transformed to frame of the cylinder
                    if norm(c2s(1:2)) < bi.radius
                        p1 = [c2s(1:2) sign(c2s(3))*bi.height/2];          % [x;y;z] Position of contact on body_1 in body_1's frame
                        p1 = quatrotate(bi.quat, p1)';                     % Transformed to world frame
                        n = sign(c2s(3)) * cz/norm(cz);                    % Normalized contact normal (colinear with cz)
                        t = arbitraryTangent(n);                           % Choose a tangent.
                        p2 = -n*bj.UNION(p).radius;                        % [x;y;z] Position of contact on body_2 in world frame
                        psi_n = norm(c2s(3)) - bi.height/2 - bj.UNION(p).radius;
                        if(psi_n < 1e-2)
                            sim.addContact(i, j, n, t, p1, p2, psi_n);
                        end
                    end
                end
            end
            
        %% Cylinder - Sphere
        elseif strcmp(bi.body_type,'cylinder') && strcmp(bj.body_type,'sphere')
            
                % 3 different "corner" cases, easily determined by
                % transforming the sphere's location into the cylinder's frame.
                cz = (bi.height/2)*quatrotate(bi.quat,[0;0;1]')';          % z vector of cylinder, with length of half the cylinder's height
                c2s = bj.u - bi.u;                                         % Vector from center of cylinder to center of sphere, world frame
                c2s = quatrotate(quatinv(bi.quat), c2s');                  % Same, but transformed to frame of the cylinder
                
                % CASE 1 - Sphere is closest to an end
                if norm(c2s(1:2)) < bi.radius
                    p1 = [c2s(1:2) sign(c2s(3))*bi.height/2];              % [x;y;z] Position of contact on body_1 in body_1's frame
                    p1 = quatrotate(bi.quat, p1)';                         % Transformed to world frame
                    n = sign(c2s(3)) * cz/norm(cz);                        % Normalized contact normal (colinear with cz)
                    t = arbitraryTangent(n);                               % Choose a tangent.
                    p2 = -n*bj.radius;                                     % [x;y;z] Position of contact on body_2 in world frame
                    psi_n = norm(c2s(3)) - bi.height/2 - bj.radius;
                    if (psi_n < 1e-1)
                        sim.addContact(i, j, n, t, p1, p2, psi_n);
                    end
                % CASE 2 - Sphere is nearest the side of the cylinder
                elseif norm(c2s(3)) < bi.height/2
                    p1 = [bi.radius*c2s(1:2)/norm(c2s(1:2)) c2s(3)];       % [x;y;z] Position of contact on body_1 in body_1's frame
                    p1 = quatrotate(bi.quat, p1)';                         % Transformed to world frame
                    n = bj.u - (bi.u+p1);                                  % Normal direction
                    n = n/norm(n);
                    t = arbitraryTangent(n);
                    p2 = -n*bj.radius;                                     % [x;y;z] Position of contact on body_2 in world frame
                    psi_n = norm(c2s(1:2)) - bi.radius - bj.radius;
                    if (psi_n < .1)
                        sim.addContact(i, j, n, t, p1, p2, psi_n);
                    end
                % CASE 3 - Sphere is nearest a corner edge (between case)
                else
                    p1a = [bi.radius*c2s(1:2)/norm(c2s(1:2)) sign(c2s(3))*bi.height/2]; % [x;y;z] Position of contact on body_1
                    p1 = quatrotate(bi.quat, p1a)';                        % Transformed to world frame
                    n = bj.u - (bi.u+p1);                                  % Normal direction
                    n = n/norm(n);
                    t = arbitraryTangent(n);
                    p2 = -n*bj.radius;                                     % [x;y;z] Position of contact on body_2 in world frame
                    psi_n = norm(c2s-p1a) * sign(dot((bj.u-bi.u)',n)) - bj.radius;
                    if (psi_n < .1)
                        sim.addContact(i, j, n, t, p1, p2, psi_n);
                    end
                end
            
        %% Cylinder - Cylinder
        elseif strcmp(bi.body_type,'cylinder') && strcmp(bj.body_type,'cylinder')
            continue; 
            % Distance between two skew cylinders
            % Method taken from last post of http://www.physicsforums.com/archive/index.php/t-13858.html
            a1 = quatrotate(bi.quat, [0 0 1])';
            a2 = quatrotate(bj.quat, [0 0 1])';
            a1ca2 = cross3(a1,a2);
            if(norm(a1ca2) ~= 0)
                n = a1ca2 / norm(a1ca2);  % Common normal between both cylinder axes
            else
                ij = bj.u - bi.u;
                n = quatrotate(quatinv(bi.quat),ij');
                n(3) = 0;
                n = quatrotate(bi.quat,n);
                n = n / norm(n);
            end
            
            t = arbitraryTangent(n);
            
            nP = cross3(n,a2);
            nP = nP/norm(nP);
            d1 = (dot((bj.u - bi.u),nP)/dot(a1,nP)); % Contact is d1 units along a1
            
            nP = cross3(n,a1); nP = nP/norm(nP);
            d2 = (dot((bi.u - bj.u),nP)/dot(a2,nP)); % Contact is d2 units along a2
            
            m = 0;
            if abs(d1) > bi.height/2 || abs(d2) > bj.height/2
                m = max(abs(d1) - bi.height/2, abs(d2) - bj.height/2);
            end
            p1 = (d1-m)*a1 + bi.radius*n;
            p2 = (d2-m)*a2 - bj.radius*n;
            
            % Now psi_n is just the distance between the contact points in the world frame
            %psi_n = abs(dot((bj.u-bi.u),n)) - bi.radius - bj.radius
            psi_n = norm(p1 + bi.u - p2 - bj.u);
            
            sim.addContact(i, j, n, t, p1, p2, psi_n);
            
            
        %% Plane - Sphere
        elseif strcmp(bi.body_type,'plane') && strcmp(bj.body_type,'sphere')
            n = bi.n; 
            psi_n = dot( n, bj.u-bi.u ) - bj.radius;
            if psi_n < 0.2                  % TODO: Unfortunately, another hard-coded epsilon
                t = arbitraryTangent(n); 
                p2 = bj.u - bj.radius*n;
                p1 = p2 - psi_n*n; 
                sim.addContact(i,j,n,t, p1-bi.u, p2-bj.u, psi_n);
            end
            
            
        %% Plane - Cylinder
        elseif strcmp(bi.body_type,'plane') && strcmp(bj.body_type,'cylinder')
            n = bi.n;                 % Normal from plane to cylinder
            t = arbitraryTangent(n);  % Tangent of plane 
            zworld = quatrotate(bj.quat,[0 0 1])';  % Cylinder's "up" vector
            h = (bj.height/2)*zworld; % Vector parallel to cylinder, half the length of its height
            
            d = dot(n,zworld); 
            if d == 1 || d==-1
                r = bj.radius*arbitraryTangent(zworld);  % Vector perp to cylinder, length of its radius
            else
                c2p = quatrotate(quatinv(bj.quat), -n'); % Negative normal in cylinder's frame 
                c2p = [c2p(1:2) 0];
                c2p = c2p/norm(c2p); 
                r = bj.radius*quatrotate(bj.quat, c2p)'; % Vector perp to cylinder, length of its radius
            end
            
            if d>0.8        % Bottom of cylinder is facing the plane
                p2 = bj.u + r - h;  
                psi_n = dot(n,p2-bi.u);
                p1 = p2-psi_n*n;  
                sim.addContact(i,j,n,t, p1-bi.u, p2-bj.u, psi_n);
                
                p2 = bj.u + rot(zworld,(2/3)*pi) * r - h;
                psi_n = dot(n,p2-bi.u);
                p1 = p2-psi_n*n;  
                sim.addContact(i,j,n,t, p1-bi.u, p2-bj.u, psi_n);
                
                p2 = bj.u + rot(zworld,-(2/3)*pi) * r - h;
                psi_n = dot(n,p2-bi.u);
                p1 = p2-psi_n*n; 
                sim.addContact(i,j,n,t, p1-bi.u, p2-bj.u, psi_n);
                
            elseif d<-0.8    % Top of cylinder is facing the plane
                p2 = bj.u + r + h;  
                psi_n = dot(n,p2-bi.u);
                p1 = p2-psi_n*n;
                sim.addContact(i,j,n,t, p1-bi.u, p2-bj.u, psi_n);
                
                p2 = bj.u + rot(zworld,(2/3)*pi) * r + h;
                psi_n = dot(n,p2-bi.u);
                p1 = p2-psi_n*n;  
                sim.addContact(i,j,n,t, p1-bi.u, p2-bj.u, psi_n);
                
                p2 = bj.u + rot(zworld,-(2/3)*pi) * r + h;
                psi_n = dot(n,p2-bi.u);
                p1 = p2-psi_n*n; 
                sim.addContact(i,j,n,t, p1-bi.u, p2-bj.u, psi_n);
                
            else            % Cylinder is mostly on its side relative to plane
                p2 = bj.u + r - h;  
                psi_n = dot(n,p2-bi.u);
                p1 = p2-psi_n*n;  
                sim.addContact(i,j,n,t, p1-bi.u, p2-bj.u, psi_n);
                
                p2 = bj.u + r + h;  
                psi_n = dot(n,p2-bi.u);
                p1 = p2-psi_n*n; 
                sim.addContact(i,j,n,t, p1-bi.u, p2-bj.u, psi_n);
            end
            
        end % END collision type check
        
    end % End for body j
        
end % End for body i


