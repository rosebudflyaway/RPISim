
% Shunk gripper (esque)

base = mesh_read_poly_file('Jenga.poly');
    base.scale(2.5);
    base.color = [.4 .4 .4];
    base.quat = qt([1;0;0],pi/2);
    %base.Fext(4) = .1;
    base.mass = 5;
    %base.setStatic( true ); 

grip1 = mesh_read_poly_file('Jenga.poly'); 
    grip1.quat = qt([0;1;0], pi/2);
    grip1.u = [-.05; 0; .04];
    grip1.mass = .2;
grip2 = mesh_read_poly_file('Jenga.poly'); 
    grip2.quat = qt([0;1;0], pi/2);
    grip2.u = [ .05; 0; .04];
    grip2.mass = .2;
    grip2.color = [1 0 0];

bodies = { base
           grip1 
           grip2 };

Sim = Simulation( bodies, 0.01 );

Sim.FRICTION = false; 
Sim.jointCorrection = false; 
Sim.controller( 'gripperController' ); % Gripper controller

% Tell grippers not to do CD on each other
grip1.doNotCollideWith(grip2.bodyID);
grip2.doNotCollideWith(grip1.bodyID);

Sim.addJoint(base, grip1, grip1.u, [ 1;0;0], 'prismatic');
Sim.addJoint(base, grip2, grip2.u, [-1;0;0], 'prismatic');

Sim.run();


