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
function results = TESTCompareSolvers( )
%% SolverHandles 
 solverHandles = {%'Lemke'
                 %'PATH'    
                  'LCP_fixed_point'
                 %'MLCPproxPGS'     % This wrapper corresponds to mlcp_fixed_point
                 %'MLCP_strict_PGS' % This wrapper corresponds to mlcp_strict_pgs                 
                 %'NCPproxPGS'      % This wrapper corresponds to mncp_fixed_point
                 %'NCP_strict_PGS'  % This wrapper corresponds to mncp_fixed_point_pgs
                 %'NCP_strict_PGS2' % This wrapper corresponds to mncp_fixed_ponit_pgs2, without update on sliding speed. 
                 %'PGS'
                 %'mpgs'
                 %'FischerNewton'
                 %'interior_point'
                 %'minmap_newton'
                 %'psor'
                 };
%% Parameters for the different error metrics, which is tunable
ErrorParams = struct();
ErrorParams.metricName = 'modified_CCK';   %fischer_burmeister   % ChenChen_Kanzow
ErrorParams.lambda1 = 0.5;
ErrorParams.lambda2 = 1;

%% Problem files, h5 file
% Initialize the problem_indicies as []; The default is empty. 
problem_indicies = [];  
%source = 1;   % 1 --- to use the h5loader 
               % 2 --- to use the HDF5loader & PROCESS & CHANGEINCONSISTENCY  
%problemFile =  'sim_data_13_04_19_19_44_03/SpheresFall.h5';
%problemFile =  'sim_data_13_04_19_19_55_19/SpheresFall.h5';               

source = 1;
%problemFile = 'h5data/logs.h5';
problemFile = 'h5data/particle_problem.h5';
% problem_indicies = (102:200);
problem_indicies = (102:200);
% source = 2;
% problemFile = 'h5data/boxes-container.h5';
% problemFile = 'h5data/spheres-no-container.h5';
% problem_indicies = (4:4);
% problem_indicies = (5:8);
% problem_indicies = (18:26);
%% Other tunable parameters 
% tol  -------------------- tolerance
% r    -------------------- r parameter in the prox function 
% SaveOrNot---------------- to save the plots directly into folder
TuneParams = struct();
% or ' '; separate the particle with the other types shape 
TuneParams.FormulationType = 'particleDynamics';  
% The formulation models 'mNCP', or 'mLCP';
%TuneParams.formulation =  'mNCP';
TuneParams.formulation = 'LCP';
TuneParams.max_iter    = 300;
TuneParams.tol = 10^(-8);
TuneParams.r = 0.31;      % 0.003
TuneParams.SaveOrNot = 0; 

%%
results = compareSolvers(solverHandles, problemFile, ErrorParams, problem_indicies, TuneParams, source);
end