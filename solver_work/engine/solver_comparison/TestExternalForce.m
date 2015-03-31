 % test file --------------> Test the convergence rate on the external
 % forces
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

function [] = TestExternalForce()
solverHandles = {
                  %'LCP_fixed_point'
                  'LCP_strict_PGS'
                };               
ErrorParams = struct();
% 1. norm_CCK, 2. square_CCK,  3. norm_mCCK  4. square_mCCK
ErrorParams.metricName = 'norm_CCK';  %'modified_CCK';   %fischer_burmeister   % ChenChen_Kanzow
ErrorParams.lambda1 = 0.7;
ErrorParams.lambda2 = 1;
source = 1;
problemFile = 'h5data/particle_problem.h5';
problem_indicies = (106:106);  % No change on the problem indicies
TuneParams = struct(); 
TuneParams.FormulationType = 'particleDynamics';  
TuneParams.formulation = 'LCP';
TuneParams.max_iter    = 300;
TuneParams.tol = 10^(-8);   
TuneParams.alpha = 10^(-18);
% The 10* is for the mass = 10
TuneParams.r = 10*(0: 0.01 : 2); %0.31;      % 0.003 
% TuneParams.r = 0.31;
%TuneParams. Forces = -10^9;  
% Make sure all the Forces are negative !!!
small = 1e-5;
TuneParams.Forces = -10*[0  5  5+small  10  1e3  1e6 1e9];

% The following lambda1 and lambda2 are related to the Newton's method,
TuneParams.lambda1 = 0.8;  % parameter used for CCK function (tunable)
TuneParams.lambda2 = 0.9;  % parameter used for modified_CCK function (tunable)

TuneParams.SaveOrNot = 0; 
results = Compare_External_Forces(solverHandles, problemFile, ErrorParams, problem_indicies, TuneParams, source); 
end