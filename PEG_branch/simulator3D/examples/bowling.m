

function sim = bowling( )

    % Initialize simulator
    sim = Simulator(0.01);      % Timestep of t = 0.01 seconds
    sim.H_dynamics = @mLCPdynamics; 
    sim.drawContacts = true; 
    sim.drawJoints = true;     
    sim.num_fricdirs = 8; 
    %sim.MAX_STEP = 500; 

    % Create an invisible static body
    staticBody = mesh_cylinder(7,1,0.2,1);
        staticBody.dynamic = false; 
        staticBody.visible = false;   % Don't bother showing the static body
        
    % Ground plane
    ground = Body_plane([0; 0; -0.7], [0; 0; 1]); 
        ground.mu = 0.2;
        ground.color = [.6 .6 .2];
        
    % Create a hanging body
    angle = pi/5; 
    hangingBody = mesh_cylinder(7,2,0.1,1);
        hangingBody.u = [-0.5*sin(angle); 0; 0.5-0.5*cos(angle)];
        hangingBody.quat = qt([0;1;0], angle);
        hangingBody.color = [.6 0 0]; 

    % A ball
    ball1 = Body_sphere(0.1, 0.1);
        ball1.u = [0;0;-.2];
    ball2 = Body_sphere(0.1, 0.1); 
        
        
    %% Gather simulation bodies and add to simulator
    bodies = [staticBody ground hangingBody ball1 ball2];
    
    % Hackish, but make sure nothing collides with the invisible body
    for b=1:length(bodies), bodies(b).doesNotCollideWith = 1; end
    bodies(1).doesNotCollideWith = 1:length(bodies);
    sim = sim_addBody( sim, bodies );

    % Create joint
    sim = sim_addJoint(sim, 1, 3, [0;0;0.25], [0;1;0], 'revolute');

    % Run simulation
    sim = sim_run( sim );

end

