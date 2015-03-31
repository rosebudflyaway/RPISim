%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
%
% Perform collision detection on a set of bodies

function sim = collision_detection( sim )

    eps_vf = 0.1;       % TODO: fix hardcoded epsilons
    eps_theta = 0.15;   % Applicability relaxation parameter
    eps_ee = 0.1;       % ...

    % Clear contact constraints
    C = [];
    [sim.bodies.active] = deal(false);  % TODO: test if this is actually more efficient 
    %for b=1:length(sim.bodies)
    %   sim.bodies(b).active = false; 
    %   sim.bodies(b).bodyContactID = 0;     % Shouldn't be necessary 
    %end
    sim.num_activeBodies = 0; 

    num_bodies = length(sim.bodies);
    for Aid = 1:num_bodies-1 
        A = sim.bodies(Aid);
        for Bid = Aid+1:num_bodies
            B = sim.bodies(Bid); 

            % Don't collide static bodies
            if ~A.dynamic && ~B.dynamic, continue; end   
            
            % Some bodies are listed as "doNotCollide" with each other,
            % e.g. bodies with joints that require overlap.
            if any(A.doesNotCollideWith == B.bodyID), continue; end

            %% Mesh - mesh
            if strcmp(A.type,'mesh') && strcmp(B.type,'mesh')
                % Broad phase (AABB check)
                if cd_intersect_AABBs(A, B)
                    
                    % Vertex-face A on B
                    c = cd_vertex_face(A,B,eps_vf,eps_theta); 
                    if ~isempty(c) 
                        sim = sim_activateBodies( sim, Aid, Bid );
                        C = [C c];
                    end
                    
                    % Vertex-face B on A
                    c = cd_vertex_face(B,A,eps_vf,eps_theta); 
                    if ~isempty(c) 
                        sim = sim_activateBodies( sim, Bid, Aid );
                        C = [C c];
                    end
                    
                    % Edge-edge
                    c = cd_edge_edge(A,B,eps_ee,eps_theta); 
                    %c = old_edge_edge(A,B); 
                    if ~isempty(c) 
                        sim = sim_activateBodies( sim, Aid, Bid );
                        C = [C c];
                    end
                end % End mesh-mesh
               
            %% Mesh - sphere
            elseif strcmp(A.type,'sphere') && strcmp(B.type,'mesh')
                if cd_intersect_sphere_AABB(A,B)
                    c = cd_sphere_polyhedron( A, B, eps_vf); 
                    if ~isempty(c) 
                        sim = sim_activateBodies( sim, Bid, Aid );
                        C = [C c];
                    end
                end
            elseif strcmp(A.type,'mesh') && strcmp(B.type,'sphere')
                if cd_intersect_sphere_AABB(B,A)
                    c = cd_sphere_polyhedron( B, A, eps_vf); 
                    if ~isempty(c) 
                        sim = sim_activateBodies( sim, Bid, Aid );
                        C = [C c];
                    end
                end
            
            %% Mesh - plane
            elseif strcmp(A.type,'mesh') && strcmp(B.type,'plane')
                c = cd_mesh_plane( A, B, eps_vf); 
                if ~isempty(c) 
                    sim = sim_activateBodies( sim, Bid, Aid );
                    C = [C c];
                end 
            elseif strcmp(A.type,'plane') && strcmp(B.type,'mesh')
                c = cd_mesh_plane( B, A, eps_vf); 
                if ~isempty(c) 
                    sim = sim_activateBodies( sim, Bid, Aid );
                    C = [C c];
                end 

            
            %% Sphere - sphere
            elseif strcmp(A.type,'sphere') && strcmp(B.type,'sphere')
                normal = B.u - A.u;
                psi_n = norm(normal) - A.radius - B.radius;
                if psi_n < eps_vf && psi_n < 0.5*A.radius   % TODO: fix hardcoded epsilons
                    normal = normal/norm(normal);
                    sim = sim_activateBodies( sim, Bid, Aid );
                    C = [C Contact(Aid, Bid, (A.u+A.radius*normal)', normal', psi_n)];
                end
                
            %% Sphere - plane
            elseif strcmp(A.type,'sphere') && strcmp(B.type,'plane')
                c = cd_sphere_plane( A, B, eps_vf); 
                if ~isempty(c) 
                    sim = sim_activateBodies( sim, Bid, Aid );
                    C = [C c];
                end 
            elseif strcmp(A.type,'plane') && strcmp(B.type,'sphere')
                c = cd_sphere_plane( B, A, eps_vf); 
                if ~isempty(c) 
                    sim = sim_activateBodies( sim, Bid, Aid );
                    C = [C c];
                end 
            
            %% Cylinder - plane
            elseif strcmp(A.type,'plane') && strcmp(B.type,'cylinder')
                n = A.plane_normal;  % Normal direction is determined by the plane
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
                    r = bj.radius*quatrotate(bj.quat, c2p)';    % Vector perp to cylinder, length of its radius
                end

                if d>0.8        % Bottom of cylinder is facing down
                    p2 = bj.u + r - h;  
                    psi_n = dot(n,p2-bi.u);
                    p1 = p2-psi_n*n;  
                    sim.Contacts = [sim.Contacts; contact(cid,i,j,n,t, p1-bi.u, p2-bj.u, psi_n)];
                    cid=cid+1; 

                    p2 = bj.u + rot(zworld,(2/3)*pi) * r - h;
                    psi_n = dot(n,p2-bi.u);
                    p1 = p2-psi_n*n;  
                    sim.Contacts = [sim.Contacts; contact(cid,i,j,n,t, p1-bi.u, p2-bj.u, psi_n)];
                    cid=cid+1; 

                    p2 = bj.u + rot(zworld,-(2/3)*pi) * r - h;
                    psi_n = dot(n,p2-bi.u);
                    p1 = p2-psi_n*n;  
                    sim.Contacts = [sim.Contacts; contact(cid,i,j,n,t, p1-bi.u, p2-bj.u, psi_n)];
                    cid=cid+1; 

                elseif d<-0.8    % Top of cylinder is facing down
                    p2 = bj.u + r + h;  
                    psi_n = dot(n,p2-bi.u);
                    p1 = p2-psi_n*n;  
                    sim.Contacts = [sim.Contacts; contact(cid,i,j,n,t, p1-bi.u, p2-bj.u, psi_n)];
                    cid=cid+1; 

                    p2 = bj.u + rot(zworld,(2/3)*pi) * r + h;
                    psi_n = dot(n,p2-bi.u);
                    p1 = p2-psi_n*n;  
                    sim.Contacts = [sim.Contacts; contact(cid,i,j,n,t, p1-bi.u, p2-bj.u, psi_n)];
                    cid=cid+1; 

                    p2 = bj.u + rot(zworld,-(2/3)*pi) * r + h;
                    psi_n = dot(n,p2-bi.u);
                    p1 = p2-psi_n*n;  
                    sim.Contacts = [sim.Contacts; contact(cid,i,j,n,t, p1-bi.u, p2-bj.u, psi_n)];
                    cid=cid+1; 

                else            % Cylinder is mostly on its side
                    p2 = bj.u + r - h;  
                    psi_n = dot(n,p2-bi.u);
                    p1 = p2-psi_n*n;  
                    sim.Contacts = [sim.Contacts; contact(cid,i,j,n,t, p1-bi.u, p2-bj.u, psi_n)];
                    cid=cid+1; 

                    p2 = bj.u + r + h;  
                    psi_n = dot(n,p2-bi.u);
                    p1 = p2-psi_n*n;  
                    sim.Contacts = [sim.Contacts; contact(cid,i,j,n,t, p1-bi.u, p2-bj.u, psi_n)];
                    cid=cid+1; 
                end

                % We are always adding contacts for now
                if bi.static == 0 
                    bi.ContactCount = bi.ContactCount + 1; 
                    if bi.BodyIndex < 1
                       bID = bID+1;
                       bi.BodyIndex = bID;  
                       sim.activeBodies = [sim.activeBodies; i];
                    end
                end
                if bj.static == 0 
                    bj.ContactCount = bj.ContactCount + 1;
                    if bj.BodyIndex < 1
                       bID = bID+1;
                       bj.BodyIndex = bID;  
                       sim.activeBodies = [sim.activeBodies; j];
                    end
                end
                

            end % End body type checks
        end
    end

    sim.contacts = C; 
    
end










