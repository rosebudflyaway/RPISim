%% scripts to read data_set and solve with different solvers
function [] = test_ode_data()
load('testme.mat');
%currentrow = 1;
for STEP  =  225 : 300    
frame_num = strcat('mbs_frame000', num2str(STEP));
cur_fm = problem_set.(frame_num);
obj.cur_fm = cur_fm;
 
% obj = read_data(obj, 'LCP');
% obj = run_onestep(obj, 'LCP', 'Lemke');

% obj = read_data(obj, 'mLCP');
% obj = run_onestep(obj, 'mLCP', 'PATH');

obj = read_data(obj,'mNCP');
obj = run_onestep(obj, 'mNCP', 'fixed_point_pgs_w');
end
end