

% Test MCP and LCP dynamics (MCP iis acting funky)

Cyl = bodyCylinder(1,2, .2);
    Cyl.u = [0;0;-.1];
    Cyl.static = true;
    Cyl.quat = quat(rand(1,3),.1);
    
S1 = bodySphere(.3, 0.3); 
    S1.name = 'S1';
    S1.u = [.2; .5; .35];
    S1.nu = [1;0;0;0;1;0];
    S1.faceAlpha = 0.3;
S2 = bodySphere(2, 0.5);
    S2.name = 'S2'; 
    S2.u = [.3; -.4; .5];

P = { Cyl
      S1 };

Sim = Simulation(P, 0.01); 
% Sim.formulation('LCP');
% Sim.solver('Lemke'); 
Sim.run();


