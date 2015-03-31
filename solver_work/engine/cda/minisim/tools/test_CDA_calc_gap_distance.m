%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
% test_CDA_calc_gap_distance.m
%
% Calculate psi - the gap distance from a vertex to an edge


function this_psi = test_CDA_calc_gap_distance(V, this_edge)


v = [this_edge(4) - this_edge(2); -(this_edge(3) - this_edge(1))];
v = v / norm(v);

r = [this_edge(1) - V.x; this_edge(2) - V.y];

this_psi = dot(v,r); % the gap distance
