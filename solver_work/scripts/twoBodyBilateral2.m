
% An attempt to mimic Jeff's twoBodyBilateral.m code for debugging

% Bodies
b1 = bodyCylinder(1,0.2,1);
    b1.J = diag([1 10 3]);
    b1.u = [-1;2;0]; 
    b1.nu = [0;0;0;0;0;0];
b2 = bodyCylinder(1,0.2,1);
    b2.J = diag([1 6 2]); 
    b2.u = [0; 1; 0];
    b2.quat = [sqrt(2)/2 0 0 sqrt(2)/2];
    b2.nu = [0;0;0;0;0;0]; 
    
% Init simulation
bodies = {b1; b2};
sim = Simulation(bodies,0.01); 
sim.gravityOFF();                   b2.Fext(5) = 1;
sim.setFriction(false);

% Add joint
sim.addJoint(b1,b2,[0;2;0],[0;0;1],'revolute'); 

% Run simulation
sim.run();



