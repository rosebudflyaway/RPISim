
% test_joint

% Create 2 bodies
body1 = bodyCylinder(1,1,10);
    body1.u = [0;0;5];
    body1.quat = quat([1;0;0],pi/2);
    body1.static = true;
    body1.color = [.6 .6 .6]; 

body2 = bodyCylinder(2,2,15); 
    body2.u = [ 0; -4-7.5*cos(pi/4);-3];
    body2.quat = quat([1;0;0],.2); 

body3 = bodyCylinder(1.5,1.5,10);
    body3.u = [0;5;0];
    body3.quat = quat([1;0;0],.2);
    
body4 = bodyCylinder(0.5,1,5);
    body4.u = [0;-11;-14];
    body4.quat = quat([1;0;0],.12);
    
% Init simulator
%bodies = {body1; body2; body3; body4};
bodies = {body1; body2; body3};
sim = Simulation(bodies,0.01);
sim.setFriction(false); 

% Add a joint
pos = [0; -8; 5];
q = [1; 0; 0]; 
sim.addJoint(body1,body2,pos,q,'revolute'); 
sim.addJoint(body1,body3,[0;6.5;5.5],q,'revolute');
%sim.addJoint(body2,body4,[0;-11;-11],q,'revolute');

sim.run();



