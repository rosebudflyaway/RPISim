

function sim = robotArm2D( dynamics, targetObject, stepSize )

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    function sim = robotArm2Dfunction( sim )
        
        if sim.step == 1
           sim.userData.errorPrev(1:4) = 0;
        end
        
        % Clear external forces on bodies
        for b=1:length(sim.bodies)
           sim.bodies(b).Fext = [0;-9.8*sim.bodies(b).mass;0];  
        end
        
        % Controller on joint torques
        target_joint_angles = [ max([-1*sim.h*(sim.step-1) -pi/2]) 
                                min([1*sim.h*(sim.step-1) pi/2]) 
                                max([-1*sim.h*(sim.step-1) -pi/2]) 
                                min([1*sim.h*(sim.step-1) pi/2])   ];
        
        
        % Joint error
        joint_errors = [  target_joint_angles(1)-sim.joints(1).theta
                          target_joint_angles(2)-sim.joints(2).theta 
                          target_joint_angles(3)-sim.joints(3).theta 
                          target_joint_angles(4)-sim.joints(4).theta    ];
                          
        d_joint_errors = [ (joint_errors(1)-sim.userData.errorPrev(1))/sim.h
                           (joint_errors(2)-sim.userData.errorPrev(2))/sim.h 
                           (joint_errors(3)-sim.userData.errorPrev(3))/sim.h 
                           (joint_errors(4)-sim.userData.errorPrev(4))/sim.h     ];
                       
        sim.userData.errorPrev = joint_errors; 
                       
        P = 5;
        D = .09;
        
        joint_torques = [ P*joint_errors(1)+D*d_joint_errors(1)
                          P*joint_errors(2)+D*d_joint_errors(2)  
                          P*joint_errors(3)+D*d_joint_errors(3)  
                          P*joint_errors(4)+D*d_joint_errors(4)  ];
        
        for j=1:4
            sim.bodies(sim.joints(j).body1id).Fext(3) = sim.bodies(sim.joints(j).body1id).Fext(3) - joint_torques(j);
            sim.bodies(sim.joints(j).body2id).Fext(3) = sim.bodies(sim.joints(j).body2id).Fext(3) + joint_torques(j);
        end
        
        
        % STORE BODY INFORMATION
        sim.userData.targetBodyPosition(sim.step,:) = sim.bodies(7).pos;
        sim.userData.targetBodyRotation(sim.step) = sim.bodies(7).rot;
        
        
        
        
%         if sim.step == 1
%            figure; hold on; 
%            sim.userData.H_joint1 = plot(sim.step,target_joint_angles(1),'r');
%            sim.userData.H_joint2 = plot(sim.step,sim.joints(1).theta(1));
%            %sim.userData.H_dtheta = plot(sim.step,d_joint_errors(1),'g'); 
%            xlabel('Time step');
%            ylabel('Joint error');
%            legend('Target \theta','Actual \theta',3);
%         else
%            x1 = get(sim.userData.H_joint1,'XData');
%            y1 = get(sim.userData.H_joint1,'YData');
%            set(sim.userData.H_joint1,'XData',[x1 sim.step]);
%            set(sim.userData.H_joint1,'YData',[y1 target_joint_angles(1)]);
%            
%            x2 = get(sim.userData.H_joint2,'XData');
%            y2 = get(sim.userData.H_joint2,'YData');
%            set(sim.userData.H_joint2,'XData',[x2 sim.step]);
%            set(sim.userData.H_joint2,'YData',[y2 sim.joints(1).theta]);
%            
%            %set(sim.userData.H_dtheta,'XData', [x1 sim.step]);
%            %set(sim.userData.H_dtheta,'YData', [get(sim.userData.H_dtheta,'YData') .1*d_joint_errors(1)]); 
%         end
        
%         if sim.step == 1
%            figure; hold on; 
%            sim.userData.H_joint1 = plot(sim.step,joint_errors(1),'r');
%            sim.userData.H_joint2 = plot(sim.step,joint_errors(2));
%            xlabel('Time step');
%            ylabel('Joint error'); 
%         else
%            x1 = get(sim.userData.H_joint1,'XData');
%            y1 = get(sim.userData.H_joint1,'YData');
%            set(sim.userData.H_joint1,'XData',[x1 sim.step]);
%            set(sim.userData.H_joint1,'YData',[y1 joint_errors(1)]);
%            
%            x2 = get(sim.userData.H_joint2,'XData');
%            y2 = get(sim.userData.H_joint2,'YData');
%            set(sim.userData.H_joint2,'XData',[x2 sim.step]);
%            set(sim.userData.H_joint2,'YData',[y2 joint_errors(2)]);
%         end
        
        
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    
    ground = body_rectangle(0.3, 0.03);
        ground.dynamic = false; 
        ground.pos = [0; .14];
        ground.color = [.6 .6 .6];
    
    base = body_rectangle(0.05, 0.01);
        base.pos = [0; .25];
        base.dynamic = false; 
        base.color = [1 0 0];
        
    rectInertia = 0.005; 
    f1 = body_rectangle(0.05, 0.01);
        f1.pos = [0.05; .25];
        f1.mass = .2;
        f1.J = rectInertia; 
    f2 = body_rectangle(0.05, 0.01);
        f2.pos = [-.05; .25];
        f2.mass = .2;
        f2.J = rectInertia; 
    f3 = body_rectangle(0.05, 0.01);
        f3.pos = [0.10; .25];
        f3.mass = .2;
        f3.J = rectInertia; 
    f4 = body_rectangle(0.05, 0.01);
        f4.pos = [-.10; .25];
        f4.mass = .2;
        f4.J = rectInertia; 
        
    % Object
%     xa = .06*rand(10,1);
%     xa = xa - mean(xa);
%     ya = .06*rand(10,1);
%     ya = ya - mean(ya); 
%     ka = convhull(xa,ya);
%     ka(end) = [];
%     targetObject = Body(xa(ka), ya(ka));  
%     targetObject.color = rand(1,3);
%     targetObject.rot = 2*pi*rand; 
%     targetObject.pos = [0; .2];
%     targetObject.mass = 0.1;
%     targetObject.J = .0001;
    
    
    %bodies = [ base f1 f2 ]; %f3 f4];
    bodies = [ground base f1 f2 f3 f4 targetObject];
    
    sim = Simulator( stepSize );
    sim.userFunction = @robotArm2Dfunction; 
    sim = sim_addBody( sim, bodies ); 
    sim.draw = true; 
    sim.drawContacts = true; 
    %sim.drawJoints = true; 
    sim.drawBoundingBoxes = false; 
    sim.FRICTION = true;                % <--------    FRICTION
    
    sim.MAX_STEP = 2/sim.h;
    %sim.MAX_STEP = 370;
    
    % Define joints
    sim = sim_addJoint( sim, 2, 3, [ .025; .25], 'revolute');
    sim = sim_addJoint( sim, 2, 4, [-.025; .25], 'revolute');
    sim = sim_addJoint( sim, 3, 5, [ .075; .25], 'revolute');
    sim = sim_addJoint( sim, 4, 6, [-.075; .25], 'revolute');
    
    % Dynamics
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

