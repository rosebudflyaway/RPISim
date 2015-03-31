%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
% cda_collide_vertex_body.m 
%

% output an active contact list for the body 1 to body 2


% (See page 112 of Binh's dissertation)
% FIXME: cite Binh's dissertation here
% Notes: (from Bihn's dissertation)
% (4.5) c = max(psi_1n - psi_2n, 0) -- p77

% use formulation in (4.11) -- p80 
%   -- p83

function [ConsPerBody C] = cda_collide_vertex_body(ConsPerBody, C, b1, b1_idx, b2, b2_idx)



% FIXME: find an intelligent maximum distance to set here
epsilon_plus = 0.5; % maximum distance (upper threshold for creating a contact)
epsilon_minus = -0.1; % minimum NEGATIVE distance (lower threshold for creating a contact)


max_psi = 0.0;
max_psi_idx = 0;


% Set the initial contact ID
num_contacts = length(C);
if (num_contacts > 1)
  cid = C(num_contacts).id + 1;
else
  cid = 1;
end




% for each vertex in body 1 ...
for incr_v = 1:b1{1}.num_verts
  
  % temporary active set of potential contacts for this vertex
  C_temp = []; % faceIDX (list of face indices)
  psi = zeros(b2{1}.num_faces,1);

  % ... compare each face in body 2 to this vertex
  for incr_f = 1:b2{1}.num_faces
  
  
    % calculate the current distance between this vertex and this face
    psi(incr_f) = cda_calc_distance_vertex_face(b1{1}, ...
                                                incr_v, ...
                                                b2{1}, ...
                                                incr_f);

    % consider this potential contact only if it's closer than the maximum distance
    % and greater than the maximum penetration depth
    if (psi(incr_f) <= epsilon_plus) && (psi(incr_f) >= epsilon_minus)
      C_temp = [C_temp; incr_f];
      
      
%        C_temp = [C_temp; contact(cid, ...
%                                  b1_idx, ...
%                                  b2_idx, ...
%                                  this_n, ...
%                                  this_t, ...
%                                  this_p1, ...
%                                  this_p2, ...
%                                  psi(incr_f))];
%        cid = cid + 1;
    end
    
    if (psi(incr_f) > max_psi)
      max_psi = psi(incr_f);
      max_psi_idx = incr_f;
    end


  end % for incr_f

  
  if (max_psi < 0.0)
    % in penetration
    
    % Get all the parameters for this contact
    this_c = cda_get_contact_params(cid, ...
                                    b1{1}, ...
                                    b1_idx, ...
                                    incr_v, ...
                                    b2{1}, ...
                                    b2_idx, ...
                                    max_psi_idx, ...
                                    max_psi);
    
    % The only constraint is the vertex in maximum penetration
    C = [C; this_c];
    
    
    % increment number of active contacts in the bodies
    cid = cid + 1;
    ConsPerBody(b1_idx) = ConsPerBody(b1_idx) + 1;
    ConsPerBody(b2_idx) = ConsPerBody(b2_idx) + 1;
    
  else
    % not in penetration
  
    % Find the potential contact with the minimum positive distance ...

    min_psi = epsilon_plus;
    min_psi_idx = 0;

    for incr_c = 1:length(C_temp)
      if (psi(C_temp(incr_c)) >= 0.0)
        if (psi(C_temp(incr_c)) <= min_psi) && (psi(C_temp(incr_c)) >= 0)
          min_psi = psi(C_temp(incr_c));
          min_psi_idx = C_temp(incr_c);
        end
      end
    end
  
  
    % ... and add it to the 'real' active set
    if (min_psi_idx > 0)
      
      % Get all the parameters for this contact
      this_c = cda_get_contact_params(cid, ...
                                      b1{1}, ...
                                      b1_idx, ...
                                      incr_v, ...
                                      b2{1}, ...
                                      b2_idx, ...
                                      min_psi_idx, ...
                                      min_psi);

            
      C = [C; this_c];
      
      % increment number of active contacts in the bodies
      cid = cid + 1;
      if (0 == b1{1}.static)
        % only counts if it's not a static body
        ConsPerBody(b1_idx) = ConsPerBody(b1_idx) + 1;
      end
      if (0 == b2{1}.static)
        % only counts if it's not a static body
        ConsPerBody(b2_idx) = ConsPerBody(b2_idx) + 1;
      end
      
      
    end
  
  end % else (max_psi < 0.0)
  
  
end % for incr_v
  
  

  
  
function c = cda_get_contact_params(cid, b1, b1_idx, v_idx, b2, b2_idx, f_idx, psi)
  
% cid - contact id
% b1 - body (vertex)
% b1_idx - index of body (vertex)
% v_idx - index of vertex
% b2 - body (face)
% b2_idx - index of body (face)
% f_idx - index of face
% psi - gap distance
  
  
  
% f_normal : Calculate face normal (world frame)
f_normal = quatrotate(b2.quat, b2.faces(f_idx).normal')';
 
% f_tangent : Calculate face tangent (world frame)
% (assuming that the first and second vertices lie in the plane of the face.)


vtx1 = b2.verts(b2.faces(f_idx).verts(1)).coords;
vtx2 = b2.verts(b2.faces(f_idx).verts(2)).coords;


f_tangent = quatrotate(b2.quat, (vtx2 - vtx1)')';

% p1 : Calculate contact point on body 1 (body_1's frame)
p1 = b1.verts(v_idx).coords;

% p2 : Calculate contact point on body 2 (body_2's frame)
% Project by distance phi down -N to the face, then transform it to body 2's frame
p1_world = quatrotate(b1.quat, p1')' + b1.u;
b2_quat_inverse = [b2.quat(1), -b2.quat(2:4)];
p2 = quatrotate(b2_quat_inverse, (p1_world + psi * -f_normal - b2.u)')';



c = contact(cid, b1_idx, b2_idx, f_normal, f_tangent, p1, p2, psi);
  
  
