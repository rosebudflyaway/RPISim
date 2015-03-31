
% PARTICLE_test2.m

pos = [0; 0; 5];     % Initial particle position
vel = [5; 0; 0];     % Initial particle velocity 
ground_radius = 13; 

% Ground
ground = obj_cylinder(1,ground_radius,1);
    ground.u = [0;0;-.5];
    ground.static = true;  
    ground.color = [.66 .71 .69];   
    
% Particles
P = {ground};
for i=.05:.05:.5
   p = obj_particle(pos);
   p.nu = vel; 
   p.Fext = [0;0;-9.8];
   p.color = rand(1,3);
   p.mu = i;
   P = [P; {p}]; 
end

% Simulate
simulator = Simulation(P,0.01); 
simulator.FORMULATION = 'particleLCP'; 
simulator.SOLVER = 'Lemke'; 
simulator.run; 

