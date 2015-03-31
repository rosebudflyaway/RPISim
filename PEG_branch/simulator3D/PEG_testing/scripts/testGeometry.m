

%% Create 2 bodies
A = mesh_icosahedron;
    A.u = [1;1;1]; 
    A.color = [.4 .4 .4]; 
    
B = mesh_octahedron;  
    B.u = [1.82;1.82;1.82];
    B.quat = qt([1;2;20],-.2);


%% Perform contact_identification / geometric tests
sim = Simulator(0.01);
sim = sim_addBody( sim, [A; B] ); 
sim.MAX_STEP = 0;
sim = sim_run( sim );


%% Draw contacts 
 %sim = get_all_contacts_3d( sim );
 %sim = peg_processContacts_3D( sim );

sim = get_constraints_by_configuration( sim );
%newNu = PEG3d( sim ); 

sim = sim_drawContacts( sim );  


