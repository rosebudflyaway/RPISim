
% A simple script for testing the simulator.  
% Simulation Test

% Objects for scene 
s1 = bodyCylinder(1,3,.5); 
    s1.static=1;
s2 = bodySphere(.5,1.5); 
    s2.u = [2;1.1;4.1];
    s2.quat = quat([0;0;1],pi/5);
s3 = bodySphere(1,.8);
    s3.u = [3;.9;2];     
    s3.static = 1;
    s3.quat = quat([0;1;0],pi/3);
s4 = bodySphere(2,.5);
    s4.u = [.8;3;3.8];
    s4.quat = quat([1;0;0],2*pi-.3);  
s5 = bodySphere(4,1);
    s5.u = [-1;1;3.5];
    

P = { s1     % Cell vector of all objects.
      s2
      s3
      s4
      s5 };

% Simulator
sim = Simulation(P,.005); 
sim.formulation('LCP');
sim.solver('Lemke'); 
sim.gravityON; 
sim.run;
