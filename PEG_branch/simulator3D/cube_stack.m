
function sim = cube_stack( )

    b1 = mesh_cube();
        b1.dynamic = false; 
        b1.color = [0 0 1];
        b1.facealpha = 1;
        %b1.quat = qt(rand(1,3),.1); 

    b2 = mesh_cube();
        b2.color = [1 0 0];
        b2.facealpha = 0.8;
        b2.u = [0;0;1.2];
        %b2 = scale_mesh(b2,1.1);

    sim = Simulator( 0.005 );
    sim = sim_addBody( sim, [b1 b2] );
    
    sim.FRICTION = false;  
    sim.drawContacts = true; 
    %sim.drawBoundingBoxes = true;  
    
    %sim.MAX_STEP = 1;
    
    dynamics = 2;
    switch dynamics
        case 1  %% Stewart-Trinkle
            sim.H_collision_detection = @collision_detection; 
            sim.H_dynamics = @mLCPdynamics; 
        case 2  %% PEG
            sim.H_collision_detection = @get_constraints_by_configuration; 
            sim.H_dynamics = @PEG3d;
    end

    sim = sim_run( sim ); 

end














