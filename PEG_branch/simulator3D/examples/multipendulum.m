

function sim = multipendulum( )

    % The angle the pendulum starts from
    angle = pi/6; 
    % Height top pendulum is attached at
    height = 2; 

    % A wall
    wall = Body_plane( [0;0;0], [1;0;0] );
    
    % A set of bobs
    b1 = Body_sphere( .5, .05 );
    b2 = Body_sphere( .5, .05 );
    b3 = Body_sphere( .5, .05 );
    
    % A set of connectors
    rod = mesh_cylinder(3, 0.1, 0.01, 0.1); 
    c1 = rod;
    c2 = rod;
    c3 = rod; 
    
    b1.u = [0;
    
    bodies = [wall c1 b1 c2 b2 c3 b3];
    
    % Init simulator
    sim = Simulator();
    sim = sim_addBody( sim, bodies );
    
    % Add joints
    sim = sim_addJoint( sim, 1, 2, 
    
    % Run simulation
    sim = sim_run( sim ); 

end

