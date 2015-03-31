
% PARTICLE_test.m
pos = [0; 0; 5];     % Initial particle position
vel = [5; 0; 0];     % Initial particle velocity 
fric_coeff = 0.2;    % Coeff of friction between particle and ground
ground_radius = 13; 
    

particle = obj_particle(pos);
    particle.nu = vel; 
    particle.Fext = [0; 0; -9.8];
    particle.mu = fric_coeff;  
    particle.color = 'red';

ground = bodyCylinder(1,ground_radius,1);
    ground.u = [0;0;-.5];
    ground.static = true;  
    ground.color = [.66 .71 .69];

simulator = Simulation({ground; particle},0.01); 
simulator.formulation('particleLCP');
%simulator.DYNAMICS = 'particle'; 
simulator.SOLVER = 'Lemke'; 
simulator.MAX_ITERS = 200;
simulator.run; 

