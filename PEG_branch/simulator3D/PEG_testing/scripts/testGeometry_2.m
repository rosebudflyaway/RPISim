

%% Create 2 bodies
A = mesh_icosahedron;
    A.u = [2.3;0;1.1]; 
    A.color = [.4 .4 .4]; 
    %A.quat = qt(rand(3,1),2*pi*rand); 
    %A.quat = qt([1;1;1],.1); 
    A.quat = qt([0;1;0],.65);
    
B = mesh_cube;  
    B = scale_mesh(B,5);
    B.u = [0;0;-2.5];
    B.dynamic = false; 
    


%% Perform contact_identification / geometric tests
sim = Simulator(0.01);
sim = sim_addBody( sim, [A; B] ); 
sim.draw = true;
sim.drawContacts = true; 
%sim.MAX_STEP = 30;


% Dynamics selection
sim.H_collision_detection = @get_constraints_by_configuration; 
sim.H_dynamics = @PEG3d;
sim = sim_run( sim );




