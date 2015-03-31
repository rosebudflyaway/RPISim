% Draw a 2D body
% Daniel Montrallo Flickinger

% DEPRECATED: use cda_obj_poly instead

function ph = cda_draw_body(b, poly_C)




% for each polygon in the body
vl_position = 1;
incr_ph = 1;

while vl_position < size(b.vcur,1)

  % the number of vertices in this polygon
  num_vertices = b.vcur(vl_position);

  % the position in vl where the vertex data for this polygon ends
  vl_position_end = vl_position + num_vertices * 3;


  % Get the current polygon
  this_poly = b.vcur(vl_position+1:vl_position_end,1);

  % Move the position in the vertex list
  vl_position = vl_position_end + 1;

  this_poly_X = [];
  this_poly_Y = [];
  this_poly_Z = [];

  % Split up the vertex data into X Y and Z
  for incr_v = 1:3:size(this_poly,1)
    this_poly_X = [this_poly_X; this_poly(incr_v,1)];
    this_poly_Y = [this_poly_Y; this_poly(incr_v+1,1)];
    this_poly_Z = [this_poly_Z; this_poly(incr_v+2,1)];
  end

  % Draw the current polygon
  ph(incr_ph) = patch(this_poly_X, this_poly_Y, this_poly_Z, poly_C);
  incr_ph = incr_ph + 1;

end

