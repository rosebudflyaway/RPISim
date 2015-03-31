%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
% test_edge_edge.m
%
    
B1 = mesh_cube();
    scl = 2;
    for i=1:B1.num_verts
      B1.verts(i).local_coords = scl*B1.verts(i).local_coords;
    end
    B1.setStatic(true); 
    B1.u = [0;0;-scl/2];
    B1.bound = 5.66; 
    B1.faceAlpha = .4;

T2 = mesh_tetrahedron();
    T2.quat = quat([1;.1;0],pi-.1);  % Flip upside down
    T2.u = [-.3 1.1 .2]';

% Pack them up
P = {B1; T2};

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Simulator
sim = Simulation(P, .01);  
sim.SOLVER = 'CDA'; 
sim.DRAW_COLLISIONS = true; 
sim.run(); 
