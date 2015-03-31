 % test file 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% INPUT: 
%   solverHandle     - A cell array of the function names as string
%
%   problemFile      - A string representing the .h5 file containing the problems. 
%
%   problem_indicies - (optional) An array of indices to use in the comparison.  
%                      If not specified, the entire problemFile will be used.
%
% OUTPUT:
%   results - A struct containing the results of comparison.  
function [] = TestCompareR()
solverHandles = {
                  %'LCP_fixed_point'
                   'LCP_strict_PGS'
                 %'NCPproxPGS'
                 };
 
% problemFile =  'sim_data_13_04_19_19_55_19/SpheresFall.h5';
                
ErrorParams = struct();
% 1. norm_CCK, 2. square_CCK,  3. norm_mCCK  4. square_mCCK
ErrorParams.metricName = 'norm_CCK';  
ErrorParams.lambda1 = 0.7;
ErrorParams.lambda2 = 1;
source = 1;
problemFile = 'h5data/particle_problem.h5';
%%%%%%%%%%%%%%%%%%%%% No change on the problem indicies %%%%%%%%%%%%%%%%%%%%%%%%%
problem_indicies = (106:106);
TuneParams = struct();
% or ' '; separate the particle with the other types shape 
TuneParams.FormulationType = 'particleDynamics';  
% The formulation models 'mNCP', or 'mLCP';
%TuneParams.formulation =  'mNCP';
TuneParams.formulation = 'LCP';
TuneParams.max_iter    = 300;
TuneParams.tol = 1e-8;
TuneParams.alpha = 0;
%TuneParams.r = (0.23 : 0.1 : 1.03); %0.31;      % 0.003
%TuneParams.r = linspace(0.1, 2.0, 50);
TuneParams.r = (0.05 : 0.01 : 1.6);
TuneParams.lambda1 = 0.8;
TuneParams.lambda2 = 0.9;

TuneParams.SaveOrNot = 0; 
 
results = Compare_R(solverHandles, problemFile, ErrorParams, problem_indicies, TuneParams, source); 
end