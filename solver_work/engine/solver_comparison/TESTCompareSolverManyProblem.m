 % test file 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% INPUT: 
%   solverHandles    - A cell array of the function names as STRINGS.
% 
%   source       - a number to decide the source of the problem files 
%
%   problemFile      - A string representing the .h5 file containing the problems. 
%
%   ErrorParams      - The parameters for different error metrics
%
%   problem_indicies - (optional) An array of indices to use in the comparison.  
%                      If not specified, the entire problemFile will be used.
%
% OUTPUT:
%   results - A struct containing the results of comparison.  
function results = TESTCompareSolverManyProblem( )
%% SolverHandles 
 solverHandles = {%'Lemke'
                 %'PATH'    
                 %'LCP_fixed_point'
                 %%'MLCPproxPGS'      % This wrapper corresponds to mlcp_fixed_point
                 %'MLCP_strict_PGS'  % This wrapper corresponds to mlcp_strict_pgs 
                 %'LCP_strict_PGS'
                 %'NCPproxPGS'       % This wrapper corresponds to mncp_fixed_point
                 %'NCP_strict_PGS'   % This wrapper corresponds to mncp_fixed_point_pgs
                 %'NCP_strict_PGS2'  % This wrapper corresponds to mncp_fixed_ponit_pgs2, without update on sliding speed. 
                  'PGS'
                 % 'FischerNewton'
                 %  'interior_point'
                 % 'minmap_newton'
                 % 'psor'
                 };
%% Parameters for the different error metrics, which is tunable
%%%%%% This is the parameters only for the errors in the updateSolutionData.m
%%%%%% The lambda in the Newton's method is different with this one and can
%%%%%% be tunable. 
ErrorParams = struct();
% 1. norm_CCK, 2. square_CCK,  3. norm_mCCK  4. square_mCCK                     
ErrorParams.metricName = 'norm_CCK';   % Chen_Chen_Kanzow
ErrorParams.lambda1 = 0.7;   % lambda1 = 1.0 ------------> Fischer Newton
ErrorParams.lambda2 = 1.0;   % lambda2 = 1.0 ------------> CCK function

%% Problem files, h5 file
problem_indicies = [];   % Initialize the problem_indicies as []; 
source = 1;
problemFile = 'h5data/particle_problem.h5';
problem_indicies = (102:162);
% source = 2;
% problemFile = 'h5data/boxes-container.h5';
% problemFile = 'h5data/spheres-no-container.h5';

%% Other tunable parameters 
% tol  -------------------- tolerance
% r    -------------------- r parameter in the prox function 
% SaveOrNot---------------- to save the plots directly into folder
TuneParams = struct();
TuneParams.FormulationType = 'particleDynamics';  
%TuneParams.formulation =  'mNCP';    % The formulation models 'mNCP', or 'mLCP';
TuneParams.formulation = 'LCP';
TuneParams.max_iter    = 300;
TuneParams.tol = 10^(-6);  
TuneParams.r = 0.9;       
TuneParams.alpha = eps;
TuneParams.SaveOrNot = 0; 

%%%% The following lambda1 and lambda2 are related to the Newton's method,
%%%% so they are tunable to find a good solution
TuneParams.lambda1 = 0.8;  % parameter used for CCK function (tunable)
TuneParams.lambda2 = 0.9;  % parameter used for modified_CCK function (tunable)
TuneParams.objectiveFunction = 'CCK'; % 1. FB  2. CCK   3. mCCK
%%
results = compareSolverManyProblem(solverHandles, problemFile, ErrorParams, problem_indicies, TuneParams, source);
end
