

function sim = box_stack( )

    N = 5;
    
    bodies = [];
    for i=1:N
       b = body_square();
       b.pos(2) = 0.5 + 1.2*i-1;
       b.color = rand(1,3); 
       bodies = [bodies b]; 
    end
    bodies(1).dynamic = false; 
        
    sim = Simulator( .1 );
    sim = sim_addBody( sim, bodies );
    
    %sim.gravity = false; 
    sim.drawContacts = true; 
    %sim.drawBoundingBoxes = true;  
    sim.MAX_STEP = 50;
    
    
    dynamics = 4;
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
    end
    
    sim = sim_run( sim ); 

end

