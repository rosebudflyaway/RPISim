%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
% obj_mesh.m 
%
% A mesh object is represented using the FACE-VERTEX format as described at
%   http://en.wikipedia.org/wiki/Polygon_mesh#Face-vertex_meshes
% Note that the mesh need not be a triangle mesh, but may have an arbitrary
% number of vertices per face.  
% 
% Represents a mesh object
classdef obj_mesh < BODY
    properties (SetAccess='public')
        % Mesh geometry variables 
        num_verts       % Number of vertices
        verts           % Ordered list of the vertices 
        num_faces       % Number of faces 
        faces           % Vector of faces making up the mesh
        num_edges       % Number of edges
        edges           % Vector of edges making up the mesh
    end
    
    methods
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %% Constructor
        %   verts is a vector of N mesh_vertex objects 
        %   faces is a vector of F mesh_face objects
        %   The attributes mesh_vertex.faces and mesh_face.verts *MUST 
        %   be assigned for all of these objects.  
        function obj = obj_mesh(verts, faces) 
            obj.body_type = 'mesh' 
            
            obj.verts = verts;      obj.num_verts = max(size(obj.verts));
            obj.faces = faces;      obj.num_faces = max(size(obj.faces));
            
            obj.edges = [];
            obj.num_edges = 0;

            % Calculate all the face normals
            for f = 1:obj.num_faces
                if (obj.faces(f).num_verts >= 3)
                    s1 = obj.verts(obj.faces(f).verts(2)).world_coords - obj.verts(obj.faces(f).verts(1)).world_coords;
                    s2 = obj.verts(obj.faces(f).verts(3)).world_coords - obj.verts(obj.faces(f).verts(2)).world_coords;
                        obj.faces(f).normal = cross3(s1, s2);
                        obj.faces(f).normal = obj.faces(f).normal / norm(obj.faces(f).normal); 
                end
                
                % Create edges while avoiding redundancies 
                for v=1:obj.faces(f).num_verts  
                    Fv1 = obj.faces(f).verts(v);       % Face vertices 
                    if v == obj.faces(f).num_verts
                        Fv2 = obj.faces(f).verts(1);
                    else
                        Fv2 = obj.faces(f).verts(v+1);
                    end
                    
                    edge_exists = false; 
                    current_edge = 1; 
                    for e=obj.num_edges:-1:1       % Compare face vertices to edge vertices
                        if Fv1 == obj.edges(e).verts(1) && Fv2 == obj.edges(e).verts(2)      % Creating edges this way works, but is very inefficient.
                                edge_exists = true;                                          % We should come up with a faster approach.  
                        elseif Fv1 == obj.edges(e).verts(2) && Fv2 == obj.edges(e).verts(1)
                                edge_exists = true; 
                        end
                        if edge_exists
                           current_edge = e;
                           break;
                        end
                    end
                    
                    % Don't add edge if it already exists, but do add the
                    % current face to it's list of faces.
                    if edge_exists
                        obj.edges(current_edge).faces(length(obj.edges(current_edge).faces)+1) = f;
                        obj.edges(current_edge).edge_angle = acos(dot( ...
                             obj.faces(obj.edges(current_edge).faces(1)).normal, ... 
                             obj.faces(obj.edges(current_edge).faces(2)).normal  )); % Calculate angle between face normals
                    % If edge doesn't exist, add it along with this face.
                    else
                        obj.edges = [obj.edges; mesh_edge([Fv1 Fv2], f)];  
                        obj.num_edges = obj.num_edges + 1; 
                    end      
                    
                end
            end % for f=1:obj.nun_faces
            
            % For each edge, assign the two values f1_ordered and
            % f2_ordered  THIS SHOULD BE ENTIRELY REDUNDANT BECAUSE OF THE
            % LOOP ABOVE FOR GENERATING EDGES.  
            for e = 1:obj.num_edges
                % Only have to look at one face, sicne f1_o and f2 o are mutually exclusive
                f = obj.faces(obj.edges(e).faces(1)); % Face 1 index on edge 1
                v1 = obj.edges(e).verts(1); % Indicies of both edge verts
                v2 = obj.edges(e).verts(2); 
                % Check the order of these verts on face f
                for fv=1:f.num_verts
                    if f.verts(fv) == v1
                        if f.verts(mod(fv,f.num_verts)+1) == v2 
                            obj.edges(e).f1_ordered = true;
                            obj.edges(e).f2_ordered = false;
                            break;
                        end
                    elseif f.verts(fv) == v2
                        if f.verts(mod(fv,f.num_verts)+1) == v1 
                            obj.edges(e).f1_ordered = false;
                            obj.edges(e).f2_ordered = true;
                            break;
                        end
                    end
                end
            end
            
            
            % Calculate radius of bounding sphere (this is practical, but not mathematically sound)
            obj.bound = 0; 
            for i=1:obj.num_verts;  
                nm = norm(obj.verts(i).local_coords); 
                if nm > obj.bound, obj.bound = nm; end
            end
            obj.bound = obj.bound*1.2;  
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %% Scale the local vertices of a mesh object 
        function obj = scale( obj, scl )
            % Scale vertices
            for i=1:obj.num_verts
                obj.verts(i).local_coords = scl*obj.verts(i).local_coords;
            end
            obj.update_world_position(); 
            
            % Scale mass ?
            obj.mass = obj.mass * scl;
            
            % Scale inertia matrix ? 
            obj.J = obj.J * scl;  
            obj.Jinv = inv(obj.J);
            
            % Recalculate radius of bounding sphere
            obj.bound = 0; 
            for i=1:obj.num_verts;  
                nm = norm(obj.verts(i).local_coords); 
                if nm > obj.bound, obj.bound = nm; end
            end
            obj.bound = obj.bound*1.2;  
        end
        
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %% setStatic()
        function obj = setStatic(obj, s)
            obj.static = s;
            if obj.static
                obj.color = [.66 .71 .69];  % I just like this color for static objs (jw)
            else
                obj.color = rand(1,3);  % Random color for dynamic objects
            end
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %% setPosition()
        function obj = setPosition(obj, p)
            if size(p,2) == 1   % The simulator expects positions to be column vectors. 
                obj.u = p;      
            else
                obj.u = p';
            end
            obj.update_world_position(); 
        end
        
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %% setAppliedForce(p,d,f)
        % p: point on body frame
        % d: world frame 1x3 direction
        % f: scalar magnitude of force to apply
        function obj = setAppliedForce(obj, p, d, f)
            
        end
        
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %% Update world frame coordinates and face normals
        function obj = update_world_position( obj )
            % Calculate world frame vertex coordinates (could really be vectorized)
            for i = 1:obj.num_verts
                obj.verts(i).world_coords = obj.u + quatrotate(obj.quat, obj.verts(i).local_coords')';
            end
            
            % Calculate all the face normals
            for incr_f = 1:obj.num_faces
                if (obj.faces(incr_f).num_verts >= 3)
                    s1 = obj.verts(obj.faces(incr_f).verts(2)).world_coords - obj.verts(obj.faces(incr_f).verts(1)).world_coords;
                    s2 = obj.verts(obj.faces(incr_f).verts(3)).world_coords - obj.verts(obj.faces(incr_f).verts(2)).world_coords;
                        obj.faces(incr_f).normal = cross3(s1, s2);
                        obj.faces(incr_f).normal = obj.faces(incr_f).normal / norm(obj.faces(incr_f).normal); 
                end
            end 
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %% Draw mesh
        function draw( obj )
            if ~obj.visible, return; end
            
            if obj.drawn == 0
%                 obj.color = rand(1,3);  
               
               % Collect all verts in a Nx3 matrix
               FV.Vertices = zeros(obj.num_verts,3);
               for i=1:obj.num_verts
                   FV.Vertices(i,1:3) = obj.verts(i).local_coords'; 
               end
               
               % Collect all faces in a FxFi matrix
               max_verts = 0;
               for i=1:obj.num_faces
                   if obj.faces(i).num_verts > max_verts
                       max_verts = obj.faces(i).num_verts;
                   end
               end
               FV.Faces = nan(obj.num_faces, max_verts);
               for i=1:obj.num_faces
                   f = obj.faces(i);
                   for j=1:f.num_verts
                      FV.Faces(i,j) = f.verts(j); 
                   end
               end
               obj.graphicsHandle = patch(FV); 
               set(obj.graphicsHandle,'FaceColor',obj.color); 
               set(obj.graphicsHandle,'FaceAlpha',obj.faceAlpha);
               set(obj.graphicsHandle,'EdgeAlpha',obj.faceAlpha); % TODO: separate face & edge alphas
               %set(obj.graphicsHandle,'LineSmoothing','on'); 
               
               % This data records the coordinates in the local frame
               obj.Xdata = get(obj.graphicsHandle,'XData');
               obj.Ydata = get(obj.graphicsHandle,'YData');
               obj.Zdata = get(obj.graphicsHandle,'ZData');
               
               obj.drawn = 1;
               
               % Get the local frame coordinates
                xd = get(obj.graphicsHandle,'XData');
                yd = get(obj.graphicsHandle,'YData');
                zd = get(obj.graphicsHandle,'ZData');
                cd = get(obj.graphicsHandle,'CData');
                set(obj.graphicsHandle,'XData',xd,'YData',yd,'ZData',zd,'CData',cd);

                % Transform data to world frame coordinates
                X = zeros(size(obj.Xdata));
                Y = zeros(size(obj.Ydata));
                Z = zeros(size(obj.Zdata));
                for i=1:length(obj.faces)
                    D = quatrotate(obj.quat, [obj.Xdata(:,i) obj.Ydata(:,i) obj.Zdata(:,i)]);
                    X(:,i) = D(:,1) + obj.u(1);
                    Y(:,i) = D(:,2) + obj.u(2);
                    Z(:,i) = D(:,3) + obj.u(3);
                end
                set(obj.graphicsHandle,'XData',X,'YData',Y,'ZData',Z);
                
            else
                
                if obj.static == 0  % Don't redraw static objects.
                    % Get the local frame coordinates
                    xd = get(obj.graphicsHandle,'XData');
                    yd = get(obj.graphicsHandle,'YData');
                    zd = get(obj.graphicsHandle,'ZData');
                    cd = get(obj.graphicsHandle,'CData');
                    set(obj.graphicsHandle,'XData',xd,'YData',yd,'ZData',zd,'CData',cd);

                    % Transform data to world frame coordinates
                    X = zeros(size(obj.Xdata));
                    Y = zeros(size(obj.Ydata));
                    Z = zeros(size(obj.Zdata));
                    for i=1:length(obj.faces)
                        D = quatrotate(obj.quat, [obj.Xdata(:,i) obj.Ydata(:,i) obj.Zdata(:,i)]);
                        X(:,i) = D(:,1) + obj.u(1);
                        Y(:,i) = D(:,2) + obj.u(2);
                        Z(:,i) = D(:,3) + obj.u(3);
                    end
                    set(obj.graphicsHandle,'XData',X,'YData',Y,'ZData',Z);
                    set(obj.graphicsHandle,'FaceColor',obj.color); 
                end
            end
        end
        
    end
end

