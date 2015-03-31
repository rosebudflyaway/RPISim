

%% Create 2 bodies
A = mesh_tetrahedron;
    A.u = [0;0;1.1]; 
    %A.color = [.4 .4 .4]; 
    A.color = [.8 .2 .2];
    %A.quat = qt(rand(3,1),2*pi*rand); 
    %A.quat = qt([1;1;1],.1); 
    %A.quat = qt([0;1;.2],.65);
    %A.nu(5) = 2;
    
B = mesh_tetrahedron;  
    %B = scale_mesh(B,5);
    B.u = [0;0;-0.5];
    B.quat = qt([1;0;0],pi); 
    B.dynamic = false; 
    B.color = [.2 .2 .8]; 
    


%% Perform contact_identification / geometric tests
sim = Simulator(0.01);
sim = sim_addBody( sim, [A; B] ); 
sim.draw = true;
sim.drawContacts = true; 
sim.MAX_STEP = 110;


% Dynamics selection
sim.H_collision_detection = @get_constraints_by_configuration; 
sim.H_dynamics = @PEG3d;
sim = sim_run( sim );




