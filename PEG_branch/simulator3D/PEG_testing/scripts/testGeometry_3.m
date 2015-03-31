

%% Create 2 bodies
A = mesh_icosahedron;
    A.u = [-0.05;0;1.1]; 
    A.color = [.4 .4 .4]; 
    %A.quat = qt(rand(3,1),2*pi*rand); 
    %A.quat = qt([1;1;1],.1); 
    %A.quat = qt([0;1;.2],.65);
    %A.nu(5) = 2;
    
B = mesh_cube;  
    B = scale_mesh(B,5);
    B.u = [0;0;-3.5];
    B.quat = qt([0;-1;0],pi/4); 
    B.dynamic = false; 
    


%% Perform contact_identification / geometric tests
sim = Simulator(0.01);
sim = sim_addBody( sim, [A; B] ); 
sim.draw = true;
sim.drawContacts = true; 
%sim.MAX_STEP = 110;


% Dynamics selection
sim.H_collision_detection = @get_constraints_by_configuration; 
sim.H_dynamics = @PEG3d;
sim = sim_run( sim );




