


function sim = tetra_prism( )

    % Static prism
    p = mesh_triprism();
        p.dynamic = false;
        p.color = [.7 .7 .7];
        p.quat = qt([0;1;0],-pi/2);
        
        
    % Tetrahedron
    t = mesh_tetrahedron();
        t.u = [0;-0.15;1.25];
        t.quat = qt([.05;1;0],pi/3.01);
        t.quat = qtmultiply( qt([0;0;1],pi/2) , t.quat );
    
    
    % Simulator
    sim = Simulator();
    sim = sim_addBody( sim, [p t] );
    sim.MAX_STEP = 100;
    sim.drawContacts = true; 
    sim.H_dynamics = @mLCPdynamics; 
    sim = sim_run( sim ); 

end

