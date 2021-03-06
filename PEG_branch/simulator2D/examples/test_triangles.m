

function sim = test_triangles( )

    T1 = body_triangle; 
        T1.color = [1 0 0];         
        %T1.rot = pi/6; 
        %T1.dynamic = false; 
    
    T2 = body_triangle;  
        %T2.pos = [0.05;3];
        %T2.pos = [3;.01]; 
        T2.pos = [2.6;0.0]; 
        %T2.rot = pi; 
        %T2.rot = pi/6;
        T2.nu(1) = -2.0; 
        %T2.Fext(1) = -1; 
        
        
%       T1.rot = .2;  T2.rot = -0.2;  T1.pos = [-.5 0];
        
    sim = Simulator( 0.05 );
    sim = sim_addBody( sim, [T1 T2] );
    
    sim.gravity = false; 
    sim.drawContacts = true; 
    %sim.drawBoundingBoxes = true;  
    
    %sim.MAX_STEP = 1;
    
    dynamics = 5;
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

