% Get a list of all the faces on a 2D object

% Daniel Montrallo Flickinger

function fl = cda_get_faces(vl)


global i_debug i_debug_fh;

% input: list of vectors
% output: list of faces

% faces list: [vertex1, vertex2, face outward normal; ...]

fl = [];
vl_pos = 1;

while vl_pos < size(vl,1)

  num_vertices = vl(vl_pos);
  

  % add two faces to close the polygon
  % FIXME: no outward normal calculated
  fl = [fl; vl(vl_pos+num_vertices*3-2:vl_pos+num_vertices*3), ...
	    vl(vl_pos+1:vl_pos+3)', ...
	    0, 0, 0];

  for incr_v = 4:3:num_vertices*3

    fl = [fl; vl(vl_pos+incr_v-3:vl_pos+incr_v-1), ...
	      vl(vl_pos+incr_v:vl_pos+incr_v+2), ... 
	      0, 0, 0];

  end % for incr_v


  vl_pos = vl_pos + 3*num_vertices + 1;

end % while vl_pos < size(vl,1)

if (i_debug > 4)
  fprintf(i_debug_fh, 'Generated %d faces from vertex list of size %d.\n', size(fl,1), size(vl,1));
end



