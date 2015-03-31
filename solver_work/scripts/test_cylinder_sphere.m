
% A simple script for testing the simulator.  
% Simulation Test

% Objects for scene 
s1 = bodyCylinder(1,1,3); 
    s1.u = [1;1;1];
    s1.quat = quat([0;1;0],pi/4-.2);
s2 = bodySphere(1,1.5); 
    s2.u = [4;1.1;4.1];
    s2.quat = quat([0;0;1],pi/5);
s3 = bodySphere(1,.3);
    s3.u = [2;.9;-1];
    s3.quat = quat([0;1;0],pi/3);
s4 = bodySphere(1,.5);
    s4.u = [.8;3;3.8];
    s4.quat = quat([1;0;0],2*pi-.3);

P = {s1     % Cell vector of all objects.
     s2
     s3
     s4};
 
% Simulator
sim = Simulation(P,.01);   
sim.run();

 