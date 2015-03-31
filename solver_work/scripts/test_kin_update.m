

% A simple script for testing the simulator.  
% Simulation Test

% Objects for scene 
s1 = bodySphere(1,.45);
    s1.u = [1;0;0];
s2 = bodySphere(1,.45); 
    s2.u = [2;0;0];
s3 = bodySphere(1,.45);
    s3.u = [4;0;0];

P = {s1     % Cell vector of all objects.
     s2
     s3};
 
% Simulator
sim = Simulation(P,.01);   
sim.run();

 

