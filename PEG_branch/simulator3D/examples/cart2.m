

function sim = cart2 ()

    sim = Simulator(.005);
    %sim.FRICTION = false; 
    %sim.num_fricdirs = 3;
    sim.H_dynamics = @mLCPdynamics;
    %sim.drawContacts = true;
    sim.drawJoints = true; 
    %sim.jointCorrection = true; 
    %sim.MAX_STEP = 0;
    
    % Ground
    ground = Body_plane([0; 0; 0],[0; .1; 1]);
        ground.color = [.7 .5 .5];
    
    % Chassis
    chassis = mesh_rectangularBlock(1,3,0.25);
        chassis.u = [0; 0; 1];
        chassis.color = [.3 .6 .5];
        %chassis.dynamic = false; 
        
    % Wheels
    w1 = Body_sphere(1,.25);  w1.mu = 1;
    w2 = Body_sphere(1,.25);  w2.mu = 1;
    w3 = Body_sphere(1,.25);
    w4 = Body_sphere(1,.25);
    w1.u = [ .5;  1.3; .75];   w1.Fext(4) = .7;
    w2.u = [-.5;  1.3; .75];   w2.Fext(4) = .7;
    w3.u = [ .5; -1.3; .75];
    w4.u = [-.5; -1.3; .75];
    
    % Add bodies to simulator
    sim = sim_addBody(sim, [ground chassis w1 w2 w3 w4]);
    
    % Create joints
     sim = sim_addJoint( sim, 2, 3, w1.u, [1;0;0], 'revolute');
     sim = sim_addJoint( sim, 2, 4, w2.u, [1;0;0], 'revolute');
     sim = sim_addJoint( sim, 2, 5, w3.u, [1;0;0], 'revolute');
     sim = sim_addJoint( sim, 2, 6, w4.u, [1;0;0], 'revolute');
    
    % Run the simulator
    sim = sim_run( sim );

end