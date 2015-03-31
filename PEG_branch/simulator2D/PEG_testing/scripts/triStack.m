

function sim = triStack( )
    
    T1 = body_triangle; 
        T1.color = [1 0 0];
        T1.pos = [-0.015; 2.5]; 
        %T1.rot = 0;
        T1.rot = -2*pi / 3;
        %T1.rot = 2*pi / 3;
    
    T2 = body_triangle;  
        T2.dynamic = false; 
        T2.rot = pi;
    
    sim = Simulator( 0.05 );
    sim = sim_addBody( sim, [T1 T2] );
    
    sim.drawContacts = true; 
    
   % sim.MAX_STEP = 42;
    
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
    end

    sim = sim_run( sim ); 


end

