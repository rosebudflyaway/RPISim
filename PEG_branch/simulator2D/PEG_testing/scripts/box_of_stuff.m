

function sim = box_of_stuff()

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    function sim = triangleSource( sim )
        
        if mod( sim.step-1, 50 ) == 0 %&& sim.step < 100
            
            % Shoots a triangle in from the source location
            source_location = [0.5; 0.8]; 
            TriScale = 0.1; 

            % Create triangle
            T = body_triangle(); 
            T.pos = source_location;
            T.rot = 4; % rand();
            
            T.verts_local = TriScale * T.verts_local; 
            T.J = TriScale^2.5 * T.J; 
            T.mass = TriScale;  
            
            T.nu(1) = -(0.5); % + rand());
            %T.nu(2) = rand(); 
            
            T.Fext = [0;-9.8*T.mass;0];

            T = body_draw_init(T);
            
            % Add body to sim
            sim = sim_addBody( sim, T );
        end
        
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    
    dw = .2;
    box_color = [.5 .5 .5]; 
    box_bottom = Body([1 1 -1 -1]', [-dw dw dw -dw]'); 
        box_bottom.dynamic = false; 
        box_bottom.pos = [0; -dw]; 
        box_bottom.color = box_color;
        box_bottom.faceAlpha = 1; 
        
    box_left = Body([dw dw -dw -dw]', [-1 1 1 -1]'); 
        box_left.dynamic = false; 
        box_left.pos = [-1; 1-2*dw];
        box_left.color = box_color;
        box_left.faceAlpha = 1; 
        
    box_right = Body([dw dw -dw -dw]', [-1 1 1 -1]');
        box_right.dynamic = false; 
        box_right.pos = [1; 1-2*dw];
        box_right.color = box_color;
        box_right.faceAlpha = 1; 
        
    sim = Simulator( 0.01 );
    sim.userFunction = @triangleSource; 
    sim = sim_addBody( sim, [box_bottom box_left box_right] ); 
    sim.drawContacts = true; 
    sim.FRICTION = false; 
    
    sim.MAX_STEP = 500;
    
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




























