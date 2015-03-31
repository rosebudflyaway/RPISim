
% Objects for scene 
T1 = mesh_tetrahedron();
    T1.u = [.6;-.9;.2];
    T1.nu = [0;0;0;0;0;-2];
    T1.quat = quat([0;0;1],1);
    T1.faceAlpha = 0.2;
    T1.mu = 0.03;
    
B1 = mesh_cube();
    scl = 2;
    for i=1:B1.num_verts
      B1.verts(i).local_coords = scl*B1.verts(i).local_coords;
    end
    B1.setStatic(true);
    B1.u = [0;0;-scl/2];
    B1.bound = 5.66; 

P = {B1; T1};

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Simulator
sim = Simulation(P, .01);  
sim.gravityON; 
sim.SOLVER = 'CDA'; 
sim.DRAW_COLLISIONS = true;
sim.run(); 

