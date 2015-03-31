%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
% mesh_edge.m 
%
% Represents the edge of a mesh object
% Every edge has the indices to the two vertices at its endpoints, and
% indices to the two faces it belongs to.  
classdef mesh_edge < handle
    properties (SetAccess='public')
        verts  % Vector of vertex indices (should only be two)
        faces  % Vector of face indices (should only be two)
        
        % For one of the faces, the vertices of this edge will be listed in
        % reverse order relative to the face (the result of each face being
        % defined in counter-clockwise order, and each edge having to
        % adjacent faces).  To reference this, we place here two booleans
        % that declare the order of faces 1 and 2.  It should always be the
        % case that one of these is true, and the other false.  
        f1_ordered
        f2_ordered
        
        normal % column vector representing normal direction (between the two faces)
        
        edge_angle  % The angle between the two face normals
    end % properties
    
    methods
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Constructor
        % Takes two verts, and two faces
        function obj = mesh_edge(verts, faces)
        
            if max(size(verts)) ~= 2
                error('Assigning more than two vertices to an edge.');
            end
            obj.verts = verts;
            
            if max(size(faces)) > 2
                error('Assigning more than two faces to an edge.'); 
            end
            obj.faces = faces;
            
            % FIXME: Calculate normal
            obj.normal = [0; 0; 0];
            
            obj.edge_angle = 0; 
            
        end % constructor
        
%         function out = eq(a, b)
%         
%         end % eq
%         
%         function disp(a)
%         
%             for incr_e = 1:length(a)
%                 
%                 disp('[[[ EDGE ]]]');
%                 disp(' ---> Vertices:');
%                 for incr_i = 1:length(a(incr_e).verts)
%                     disp(a(incr_e).verts(incr_i));
%                 end
%                 if (0 == length(a(incr_e).verts))
%                     disp('   * dust *');
%                 end
%         
%                 disp(' ---> Faces:');
%                 for incr_i = 1:length(a(incr_e).faces);
%                     disp(a(incr_e).faces(incr_i));
%                 end
%                 if (0 == length(a(incr_e).faces))
%                     disp('    * dust *');
%                 end
%             end
%         
%         end % disp
        
    end % methods
end % classdef
