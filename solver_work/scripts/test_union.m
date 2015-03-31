function [] = test_union()

r = 0.1;
mass = 1;
num = 6;

posi1 = [0; 0.21; 2.3];
posi2 = [0; 0.32; 2.9]; 
posi3 = [0; 0.06; 2.7];
un1 = obj_union(mass, r, num, posi1); 
un1.static = 1;

un2 = obj_union(mass, r, num, posi2); %obj_union(mass, radius, num_of_spheres)    
un2.Fext_constant = [0;0;-980*6;0;0;0];

un3 = obj_union(mass, r, num, posi3); %obj_union(mass, radius, num_of_spheres)    
un3.Fext_constant = [0;0;-980*6;0;0;0];

P = {un1; un2; un3};
obj = Simulation(P, 0.001);
obj.formulation('mLCP');
obj.solver('PATH');
obj.run();
end