%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
%
% Runs a simulator 

function sim = sim_run( sim )

    % Initialize graphics 
    sim = sim_draw_init( sim );
    if sim.draw 
        disp('Press the "any key" to start simulating.'); 
    else
        disp('Simulating without GUI, make sure you have set sim.MAX_STEP to a reasonable value.'); 
    end
    
    % Turn "on" gravity
    if sim.gravity, sim = sim_setGravity( sim ); end
    
    % Currently, the LCP formulation doesn't handle bilateral constraints,
    % so check this for user's sanity.
%     if ~isempty(sim.joints) && ~isequal(sim.H_dynamics, @mLCPdynamics)
%        disp(' *** So sorry, but joints are only supported when using @mLCPdynamics. ***');
%        error('This can be fixed by setting sim.dynamics=@mLCPdynamics before running.');
%     end
    
    %pause(); 
    
            %writerObj = VideoWriter('PEG_triangles.avi');
            %open(writerObj);

    %% Simulation loop 
    tic;
    sim.START_TIME = tic; 
    for timestep = 1:sim.MAX_STEP 
        tic;                                % Start step timer
        sim.time = sim.time + sim.h;
        sim.step = sim.step + 1;
        
        % Run user-defined function
        if ~isempty(sim.userFunction), sim = feval( sim.userFunction, sim ); end

        % Generate a contact set, and identify participating dynamic bodies 
        if ~isempty( sim.H_collision_detection )
            sim = feval( sim.H_collision_detection, sim ); 
        end

        newNU = [];
        % Formulate dynamics and solve for new body velocities
        if ~isempty(sim.contacts) || sim.num_jointConstraints > 0   
            if ~isequal(sim.H_dynamics,@PEGdynamics)  
                sim = preDynamics( sim );        
            end
            %[newNU,sim] = feval( sim.H_dynamics, sim );  
            newNU = feval( sim.H_dynamics, sim );
        end
        
        sim.userData.Ucount(sim.step) = length(sim.Uconstraints);
        sim.userData.Icount(sim.step) = length(sim.Iconstraints);
        sim.userData.Xcount(sim.step) = length(sim.Xconstraints); 
        sim.userData.contactCount(sim.step) = length(sim.contacts); 
         
        % Apply results from solution, as well as update bodies not in contact.   
        sim = body_updateAllBodies(sim,newNU);
        
        % Correct joint errors
        if sim.jointCorrection, sim = joint_correction( sim ); end
        
        % Update title and graphics
        if sim.draw
            figure(sim.figure);
%             title(['Timestep (' num2str(length(sim.contacts)) ' contacts): ' num2str(sim.step)]);
            title(['Timestep: ' num2str(sim.step)]);
            if sim.drawContacts, sim = sim_drawContacts(sim); end
            if sim.drawJoints,   sim = sim_drawJoints(sim);   end
            %axis equal;
            drawnow; 
            if sim.step == 1
                set(gcf,'units','normalized','outerposition',[0 0 1 1]);
            end
        else
            disp(['Step ' num2str(sim.step) ' took ' num2str(toc) ' seconds']);
        end
        
        % Record data for playback 
        if sim.record, sim = sim_record(sim); end 
        
%                 %axis([-3 5.5 -4 6]);
%                 grid off; 
%                 %frame = getframe;
%                 sim.M(sim.step) = getframe; 
%                 %writeVideo(writerObj,frame);
%                 
%                 
%                 if sim.step > 79
%                    2;  
%                 end
                
        
    end % End simulation loop
    sim.TOTAL_TIME = toc(sim.START_TIME); 

    disp(['Simulation finished!  Total time: ' num2str(sim.TOTAL_TIME) ' seconds.']);
    
end % End sim_run()












