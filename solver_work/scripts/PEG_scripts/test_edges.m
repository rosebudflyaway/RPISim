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
    T1.color = [0.2 0.2 0.6];
T2 = mesh_tetrahedron();
    T2.color = [0.0 0.6 0.0];
    T2.static = 0;
     T2.quat = quat([1;0;0],pi-.1);  % Flip upside down
    T2.u = [0.35 -.1 1.2]';
    %T2.u = [.3 -.6 1.5]';
    %T2.u = [.3; -.6; 2]
    
% Pack them up
P = {T1; T2};

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%T = 0.0; % current simulation time
TFINAL = 0.01; % final simulation time
TSTEP = 0.005; % simulation time step

% Simulator
sim = Simulation(P, TSTEP); 
sim.gravityON; 
sim.SOLVER = 'CDA'; 
sim.run();

