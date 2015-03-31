

function sim = box_stack( dynamics )


    N = 10;
    
    bodies = [];
    for i=1:N 
       b = body_square();
       %b.pos(1) = 0.3-.3*rand;
       if mod(i,2) == 0
          b.pos(1) = -.02-.051*rand;
       else
           b.pos(1) = .02+.051*rand;
       end
       b.pos(2) = 0.5 + 1.2*i-1;
       b.color = rand(1,3); 
       bodies = [bodies b]; 
    end
    bodies(1).dynamic = false; 
    
    %save('box_stack_bodies','bodies'); 
    load('box_stack_bodies'); 
        
    sim = Simulator( .002 );
    sim = sim_addBody( sim, bodies );
    
    %sim.gravity = false; 
    sim.drawContacts = true; 
    %sim.drawBoundingBoxes = true;  
    sim.MAX_STEP = 500;
    sim.FRICTIOn = false; 
    
    
    %dynamics = 1;
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

