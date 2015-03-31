

function sim = tri_drop( dynamics, timeStep )

    

    function sim = collectData( sim )
       
        sim.userData.position(sim.step,:) = sim.bodies(2).pos'; 
        sim.userData.rotation(sim.step) = sim.bodies(2).rot; 
        
    end

    
    T1 = body_triangle;  
        T1.dynamic = false; 
        T1 = body_scale(T1,2); 
        
    T2 = body_triangle; 
        T2.color = [1 0 0];
        T2.pos = [1; 5]; 
        T2.rot = 0;
        T2.rot = pi / 3;
    
    sim = Simulator( timeStep );
    sim = sim_addBody( sim, [T1 T2] );
    sim.FRICTION = false; 
    sim.userFunction = @collectData; 
    
    sim.draw = false; 
    %sim.drawContacts = true; 
    
    sim.MAX_STEP = 1.5/timeStep;
    
    %dynamics = 5;
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

