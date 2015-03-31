% A simple script for testing the simulator.  
% Simulation Test

% Objects for scene 
p1 = bodySphere(1,2); p1.u = [.2;0;0]; p1.static=1; %p1.nu = [-.3;0;0;0;0;0];
    p1.mu = .5;
p2 = bodySphere(1,1); 
    p2.u = [0;0;3]; 
    p2.nu = [0;0;0;0;0;0];  
    p2.color = rand(1,3); 
    p2.mu = .5;

p3 = bodySphere(1,1); p3.u=[-.2;0;5.4]; 

P = {p1     % Cell vector of all objects.
     p2
     p3 };
 
% Simulator
sim = Simulation(P,.01);
% sim.formulation('LCP'); 
% sim.solver('Lemke');

%sim.formulation('mLCP');
%sim.solver('mlcp_fixed_point');

%sim.formulation('mNCP');
%sim.solver('mncp_fixed_point_pgs');
%sim.solver('mncp_fixed_point_pgs');
sim.run();
