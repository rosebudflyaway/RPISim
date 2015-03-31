%% scripts to read data_set and solve with different solvers
% To keep consistent with the simulator, the iter and err information will
% be saved in the obj.solution.err and obj.solution.iter
function [] = run_scripts()
%data_source = 'simulator';
data_source = 'ode';

if strcmp(data_source, 'simulator')
    load('data_set_Lemke.mat');
else
    data_set = h5load('chain.h5');
end
% load('data_set_PATH.mat');
currentrow = 1;
comparison = zeros(76, 10);
data_set
length(fieldnames(data_set))
%for STEP  =  225 : 300
for STEP = 1 : length(fieldnames(data_set))
    disp(STEP);
    frame_num = sprintf('frame%06d', STEP);
    %frame_num =  num2str(FrameName);
    cur_fm = data_set.(frame_num);
    obj.cur_fm = cur_fm;
    % obj = read_data(obj, 'LCP');
    % obj = run_onestep(obj, 'LCP', 'Lemke');
    % iter1 = obj.iter
    % err1 = obj.err
    
    % obj = read_data(obj, 'mLCP');
    % obj = run_onestep(obj, 'mLCP', 'PATH');
    % err2 = obj.err;
    % iter2 = 0;
    
    % obj = read_data(obj,'mNCP');
    % obj = run_onestep(obj, 'mNCP', 'fixed_point_pgs');
    % iter3 = obj.iter;
    % err3 = obj.err;
    
    %  obj =  read_data(obj, 'LCP');
    %  obj = run_onestep(obj, 'LCP', 'FischerNewton');
    %  iter4 = obj.iter;
    %  err4 = obj.err;
    
    % obj = read_data(obj, 'LCP');
    % obj = run_onestep(obj, 'LCP', 'PGS');
    % iter5  = obj.iter;
    % err5 = obj.err;
    %% for different kind of fixed point method with prox formulation
    obj.cur = read_data(data_set.(frame_num), 'mLCP');
    obj.cur = run_onestep(obj, 'mLCP', 'fixed_point');
    iter1 = obj.solution.iter;
    err1 = obj.solution.err;
    
    obj = read_data(obj, 'mNCP');
    obj = run_onestep(obj, 'mNCP', 'fixed_point_blockpgs_w');
    err2 = obj.solution.err;
    iter2 = obj.solution.iter;
    
    obj = run_onestep(obj, 'mNCP', 'fixed_point_blockpgs_wo');
    err3 = obj.solution.err;
    iter3 = obj.solution.iter;
    
    obj = run_onestep(obj, 'mNCP', 'fixed_point_pgs_w');
    iter4 = obj.solution.iter;
    err4 = obj.solution.err;
    
    obj = run_onestep(obj, 'mNCP', 'fixed_point_pgs_wo');
    iter5 = obj.iter;
    err5 = obj.err;
    
    comparison(currentrow, :) = [iter1, err1, iter2, err2, iter3, err3, iter4, err4, iter5, err5];
    %     %iter4 max(err4) iter5 max(err5)];
    currentrow = currentrow + 1;
    %
end
save('solver_comparision_results/solver_comparison_2.mat', 'comparison');
end