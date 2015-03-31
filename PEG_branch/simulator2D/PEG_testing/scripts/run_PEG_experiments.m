



%load('experiments_02/body_sets/random_bodies_d.mat');
%tempB1 = B;
%load('experiments_02/body_sets/random_bodies_j.mat');
%tempB2 = B;
%B = [tempB1 tempB2]; 



load('experiments_02/body_sets/random_bodies_a.mat');
for b=1:length(B), B(b).numJoints = 0; end
sim = box_of_poly( 1, 0.005, B );
percentST = sim.userData.STcount; 
%save('sim_005_a_STcount2.mat','percentST');






% PEG
% load('experiments_02/body_sets/random_bodies_a.mat');
% sim = box_of_poly( 6, 0.001, B );
% err = sim.userData.polyError; 
% save('experiments_02/penalty/pen_sim_001_a.mat','err');
% 
% load('experiments_02/body_sets/random_bodies_a.mat');
% sim = box_of_poly( 6, 0.002, B );
% err = sim.userData.polyError; 
% save('experiments_02/penalty/pen_sim_002_a.mat','err');
% 
% load('experiments_02/body_sets/random_bodies_a.mat');
% sim = box_of_poly( 6, 0.003, B );
% err = sim.userData.polyError; 
% save('experiments_02/penalty/pen_sim_003_a.mat','err');

    
    
    
    
