 %% scripts to read data_set and solve with different solvers
% To keep consistent with the simulator, the iter and err information will
% be saved in the obj.solution.err and obj.solution.iter
function[] = errorCompareRunMe()
%data_source = 'simulator';
data_source = 'ode';
if strcmp(data_source, 'simulator')
    load('data_set_Lemke.mat');
else
    % data_set = h5load('chain.h5');
    disp('Loading hdf5 file...'); 
    data_set = h5load('logs.h5');
    disp('Done.');
end
% TotalStep = length(fieldnames(data_set));
for STEP = 30 : 32
%for STEP = 1 : 3
    disp(STEP);
    CurrentFrame = sprintf('frame%06d', STEP);
    % data_set.(frame_num) has the property of bodies, contacts,
    % constraints, dynamics and solution.     
   
    %% mNCP formulation
    % compute the predynamics 
    data_set.(CurrentFrame).dynamics = struct();
    data_set.(CurrentFrame).dynamics = PreDynamicsWFormulation(data_set.(CurrentFrame), 'mNCP');
    % each solution includes the info: iter,  err[iter],  slideNUM[iter],  normErr[iter], FricErr[iter],    
    
    % call the solver and get the solution 
    solverName = 'fixed_point_pgs_w';
    data_set.(CurrentFrame).solution.FixedPGSNCPSoln = struct();
    data_set.(CurrentFrame).solution.FixedPGSNCPSoln = SubProblem(data_set.(CurrentFrame), 'mNCP', solverName); 
    
    %% mLCP formulation 
    % compute the predynamics 
    data_set.(CurrentFrame).dynamics = struct();
    data_set.(CurrentFrame).dynamics = PreDynamicsWFormulation(data_set.(CurrentFrame), 'mLCP');
    % call the solver and get the solution 
    solverName = 'PATH';
    data_set.(CurrentFrame).solution.PathSoln = struct();   
    data_set.(CurrentFrame).solution.PathSoln = SubProblem(data_set.(CurrentFrame), 'mLCP', solverName); 
    solverName = 'fixed_point';
    data_set.(CurrentFrame).solution.FixedMLCPSoln = struct();
    data_set.(CurrentFrame).solution.FixedMLCPSoln = SubProblem(data_set.(CurrentFrame), 'mLCP', solverName);
    
    %% pure LCP formulation 
    % compute the predynamics 
    data_set.(CurrentFrame).dynamics = struct();
    data_set.(CurrentFrame).dynamics = PreDynamicsWFormulation(data_set.(CurrentFrame), 'LCP');
    % call the solver and get the solution 
%     solverName = 'Lemke';
%     data_set.(CurrentFrame).solution.LemkeSoln = struct();   
%     data_set.(CurrentFrame).solution.LemkeSoln = SubProblem(data_set.(CurrentFrame), 'LCP', solverName); 
    
    solverName = 'PGS';
    data_set.(CurrentFrame).solution.PGSSoln = struct();   
    data_set.(CurrentFrame).solution.PGSSoln = SubProblem(data_set.(CurrentFrame), 'LCP', solverName); 
    
    solverName = 'FischerNewton';
    data_set.(CurrentFrame).solution.FischerNewtonSoln = struct();   
    data_set.(CurrentFrame).solution.FischerNewtonSoln = SubProblem(data_set.(CurrentFrame), 'LCP', solverName); 
end
    FileName = strcat('DataSet_', datestr(now, 'yy_mm_dd_HH_MM'),  '.mat');
    save(FileName, 'data_set');
end