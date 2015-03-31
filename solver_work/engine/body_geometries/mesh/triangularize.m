%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
% triangularize.m 
%
% Given a mesh object m, returns an equivalent triangle mesh.  

function trimesh = triangularize(m) 
  
  % Let's first remove all edges, and remove all face pointers from verts.
  % This isn't the most efficient algorithm, but is easy and works.
  m.edges = [];
  for v=1:m.num_verts
     m.verts(v).faces = [];
  end

  faces = [];
  % For every face
  for f=1:m.num_faces
     % If the face is triangular
     if m.faces(f).num_verts == 3
        % Add the single face
        faces = [faces; m.faces(f)]; 
     else
        % Divide into triangle faces
        for v=3:m.faces(f).num_verts
            faces = [faces; mesh_face( [ m.faces(f).verts(1)
                                         m.faces(f).verts(v-1) 
                                         m.faces(f).verts(v)] ) ];
        end
     end
  end
  
  trimesh = obj_mesh(m.verts,faces);  % Return the triangle mesh

end 

