


b1 = mesh_icosahedron();
    b1.color = [1 0 0];
    b1.u = -2*rand(1,3);                % Random position
    b1.quat = qt(rand(3,1),10*rand);    % Random rotation
    b1.dynamic = false; 
    
b2 = mesh_icosahedron();
    b2.u = -2*rand(1,3) + [0;0;2]';
    b2.quat = qt(rand(3,1),10*rand);



sim = Simulator();
sim.drawContacts = true; 
sim.MAX_STEP = 0; 

sim = sim_addBody( sim, [b1 b2] );
sim = sim_run( sim );





