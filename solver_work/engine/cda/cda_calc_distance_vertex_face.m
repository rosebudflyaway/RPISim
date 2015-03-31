%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
% mesh_face.m 
%
% calculate the distance between a vertex and face
%  http://mathworld.wolfram.com/Point-PlaneDistance.html


function psi = cda_calc_distance_vertex_face(b1, vi, b2, fi)

% b1 : body 1 (vertex)
% vi : vertex index

% b2 : body 2 (face)
% fi : face index

% v: vertex object as defined in mesh_vertex.m
% f: face object as defined in mesh_face.m

% Calculate the vertex of body 1 in the worldframe
v =  quatrotate(b1.quat, b1.verts(vi).coords')' + b1.u;

% Calculate the face normal in the worldframe
f_normal = quatrotate(b2.quat, b2.faces(fi).normal')';

% Calculate the first vertex in the face in the worldframe
f_vertex = quatrotate(b2.quat, b2.verts(b2.faces(fi).verts(1)).coords')' + b2.u;

% Project the distance from any point on the plane of the face along the face normal direction
psi = f_normal(1) * (v(1) - f_vertex(1)) + ...
      f_normal(2) * (v(2) - f_vertex(2)) + ...
      f_normal(3) * (v(3) - f_vertex(3)); 

