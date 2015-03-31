

%% Define some bodies
% Note: mesh attributes are listed in obj_mesh.m  
C = mesh_cube;  
    C.scale(5);          % Scale the body
    C.u = [0; 0; -2.5];  % Set its position
    C.setStatic(true);     % Set as a static body

T1 = mesh_tetrahedron;
    T1.u = [1; 1; 3]; 
    T1.quat = quat(rand(3,1), rand);    % Set random rotation
    T1.Fext = [0;0;-9.8*T1.mass;0;0;0]; % Set external forces
    T1.color = [.3 .7 .5];
    
% T2 = mesh_octahedron;
%     T2.u = [-1; 2; 4];
%     T2.quat = quat([1; 0.0; 0], .3);    % Set specific rotation
%     T2.Fext = [0;0;-9.8*T2.mass;0;0;0];
T2 = mesh_octahedron;
    T2.u = [-1.3; 2; 1.2];
    T2.quat = quat([1; 0.1; 0], .8);    % Set specific rotation
    T2.Fext = [0;0;-9.8*T2.mass;0;0;0];
    T2.nu = [-.3;0;0;0;0;0];
    
%% Setup the simulator
bodies = {C; T1; T2};
%bodies = {C; T1};
step_size = 0.01; 
sim = Simulation(bodies, step_size);
sim.formulation('PEG'); 
%sim.setGravity(-9.81);  % Not yet implemented 


%% Simulate
sim.run();  


