

%% Define some bodies
% Note: mesh attributes are listed in obj_mesh.m  
C = mesh_cube;  
    C.scale(5);          % Scale the body
    %C.u = [0; 0; -2.5];  % Set its position
    C.setStatic(true);     % Set as a static body

T1 = mesh_tetrahedron;
%T1 = mesh_cube;
    T1.u = [2; 0; 3]; 
    %T1.u = [0; 0; 3]; 
    %T1.quat = qt(rand(3,1), rand);    % Set random rotation
    T1.nu(1) = -1;
    T1.mu = 0; 
    T1.Fext = [0;0;-9.8*T1.mass;0;0;0]; % Set external forces
    T1.color = [.3 .7 .5];
  
    
%% Setup the simulator
bodies = {C; T1};
step_size = 0.01; 
sim = Simulation(bodies, step_size);
sim.formulation('PEG'); 
sim.useBULLET = true; % Bullet Collision Detection

%% Simulate
sim.run();  
