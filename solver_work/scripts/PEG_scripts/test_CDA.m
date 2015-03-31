%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
% test_CDA.m
%  
% Test of the simulator with the CDA algorithms enabled
%

% Objects for scene 
T1 = mesh_tetrahedron();
    T1.static = 1;
    T1.color = [0.1 0.1 1];
    T1.faceAlpha = 0.5; 
    T1.u = [.05;.05;.1];
    
B1 = mesh_cube();
    scl = 5;
    for i=1:B1.num_verts
      B1.verts(i).local_coords = scl*B1.verts(i).local_coords;
    end
    B1.setStatic(true);
    B1.u = [0;0;-scl/2];
    B1.bound = 5.66; 

T2 = mesh_tetrahedron();
    T2.static = 0;
    %T2.quat = quat([1;0;0],pi-.1);  % Flip upside down
    T2.quat = quat([.9;.1;.3],.2);
    T2.u = [.1 .1 1.5]';
    %T2.u = [0.3 -.6 1.5]';
    T2.color = [0.0 0.6 0.0];
    T2.Fext(3) = -9.8;
    
T3 = mesh_tetrahedron();
    T3.static = 0;
    %T3.quat = quat([1;0;0],pi);  % Flip upside down
    T3.quat = quat([0;1;1],.2);
    T3.u = [-.1 .7 3]';
    %T3.nu = [0 .5 0 0 0 0]';
    T3.color = [0.7 0.1 0.4];
    T3.Fext(3) = -9.8;
    
T4 = mesh_tetrahedron();
    T4.u = [-2.2;-.5;.4];
    T4.nu = [-.5;0;0;0;0;0];
    T4.quat = quat([0;.2;1],.5);

% Pack them up
%P = {T1; T2};
P = {B1; T2; T3};
%P = {B1; T4};

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Simulator
sim = Simulation(P, .01);  
sim.formulation('PEG');
sim.num_fricdirs = 0;
% sim.MAX_ITERS = 100; 
% sim.DRAW = false; 
% sim.RECORD_DATA = true;
sim.run(); 



