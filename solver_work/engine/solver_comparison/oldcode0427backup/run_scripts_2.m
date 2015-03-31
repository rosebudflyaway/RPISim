function [] = run_scripts_2()
% this is to test with only MNCP fixed-point method to make it work first,
% then go to the run_scripts to make different methods work. 
load('testme.mat');
num_steps = length(fieldnames(problem_set));

for STEP  =  125 : 525    
frame_num = strcat('mbs_frame000', num2str(STEP));
cur_fm = problem_set.(frame_num);
obj.cur_fm = cur_fm;
obj.cur_fm.settings.h = 0.001;

obj = read_data(obj, 'LCP');
obj = run_onestep(obj, 'LCP', 'Lemke');
iter1 = obj.iter;  err1 = obj.err;

 
obj = read_data(obj, 'mLCP');
obj = run_onestep(obj, 'mLCP', 'PATH');
iter2 = obj.iter;  err2 = obj.err; 

obj = read_data(obj,'mNCP');
obj = run_onestep(obj, 'mNCP', 'fixed_point_pgs');
iter3 = obj.iter;  err3 = obj.err;
%  obj =  read_data(obj, 'LCP');
%  obj = run_onestep(obj, 'LCP', 'FischerNewton');
%  %z4 = obj.z; 
%  iter4 = obj.iter; err4 = obj.err;

%comparison(STEP, :) = [z1 iter1 err1 z2 iter2 err2 z3 iter3 err3];
% comparison to save the information of different solvers and do comparison
comparison(STEP, :) = [iter1 err1 iter2  err2 iter3 err3];
comparison
end

end