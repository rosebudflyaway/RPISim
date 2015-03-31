%% temporary scripts to test the new error metric for different 
%  solvers one by one, just temporary
function [] = run_scripts_one()
data_source = 'ode';
%data_source = 'simulator';
if strcmp(data_source, 'simulator')
    load('data_set.mat');
else
    data_set = h5load('chain.h5');
end 
fields = fieldnames(data_set);
length(fields)
converge_data = zeros(1000, length(fields));
%size(converge_data)
 for STEP  =  1 : length(fields) 
% for STEP  =  1 : 2
frame_name = sprintf('frame%06d', STEP);
%frame_num = strcat('frame', num2str(STEP));
%data_set.(frame_name) = data_set.(frame_name);
data_set.(frame_name).solution = struct();
data_set.(frame_name).dynamics = struct();
data_set.(frame_name) = read_data(data_set.(frame_name), 'mNCP');
data_set.(frame_name) = run_onestep(data_set.(frame_name), 'mNCP', 'fixed_point_pgs_w');
% temp2 = data_set.(frame_name).solution.converge;
%size(converge_data)    
end
%save( 'converge_data.mat', 'converge_data');
%data_set
save('data_set_with_solution.mat', 'data_set');
% data_set
% data_set.frame000001.dynamics
% data_set.frame000002.solution
% normal error here is saved per contact, per iteration.
% Say we have m contacts, n iterations, then the size of the normerr matrix
% is m X n
end

