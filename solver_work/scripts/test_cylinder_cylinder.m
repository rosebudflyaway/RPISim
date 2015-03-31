
% A simple script for testing the simulator.  
% Simulation Test

% Objects for scene 
s1 = obj_cylinder(1,1,5); 
    s1.u = [1;-.4;-1];
    s1.quat = quat([1;0;0],0);
s2 = obj_cylinder(1,1,4); 
    s2.u = [4;0;0];
    s2.quat = quat([0;1;0],0);

P = {s1     % Cell vector of all objects.
     s2
     };
 
% Simulator
sim = Simulation(P,.01);   
sim.DRAW_VECS = 1;

% Simulate
figure(1); hold on; axis equal; grid on;
xlabel('X'); ylabel('Y'); zlabel('Z');
sim.step(); 


%for iter = 1:360
%    view( -37.5 + iter , 25 );
%    pause(0.05);
%end

view(3);

 