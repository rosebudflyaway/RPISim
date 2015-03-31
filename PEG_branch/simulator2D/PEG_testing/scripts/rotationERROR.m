

function sim = rotationERROR(  )

    R1 = body_rectangle(2,0.5);  
        R1.pos = [0; -.25];
        R1.nu(3) = 1;
        
    R2 = body_rectangle(1,0.5); 
        R2.color = [1 0 0];
        R2.pos = [0; .25]; 
        R2.nu(3) = 1; 
    
    sim = Simulator( 0.5 );
    sim = sim_addBody( sim, [R1 R2] );
    sim.FRICTION = false; 
    
    %sim.drawContacts = true; 
    sim.gravity = false; 
    sim.MAX_STEP = 0;
    
    dynamics = 1;
    switch dynamics
        case 1  %% Stewart-Trinkle
            sim.H_collision_detection = @collision_detection; 
            sim.H_dynamics = @mLCPdynamics; 
        case 2  %% PEG
            sim.H_collision_detection = @peg_collision_detection; 
            sim.H_dynamics = @PEGdynamics; 
        case 3  %% mPEG
            sim.H_collision_detection = @mPEG_collision_detection; 
            sim.H_dynamics = @mPEGdynamics; 
        case 4 %% NEW PEG
            sim.H_collision_detection = @get_all_contacts_2d;
            sim.H_dynamics = @PEG_dynamics; 
        case 5 %% Heuristic PEG
            sim.H_collision_detection = @get_all_contacts_2d;
            sim.H_dynamics = @PEG2_dynamics; 
        case 6 %% Penalty method
            sim.H_collision_detection = @penaltyCD;
            sim.H_dynamics = @mLCPdynamics; 
    end
    sim = sim_run( sim ); 

end

