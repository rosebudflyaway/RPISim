
% Compares correction with no correction


% num_bodies = 3; 
% sim1 = snake_joints(num_bodies,true,100,false);
% plot_joint_error(sim1); title('With Joint Correction');
% sim2 = snake_joints(num_bodies,false,100,false);
% plot_joint_error(sim2); title('No Correction');


sim1 = hanging_joint(true,10000,false);
plot_joint_error(sim1); title('With Joint Correction');
sim2 = hanging_joint(false,10000,false);
plot_joint_error(sim2); title('No Correction');




