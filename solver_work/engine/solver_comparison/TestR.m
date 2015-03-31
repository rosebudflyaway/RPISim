 % test file 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% INPUT: 
%   solverHandles    - A cell array of the function names as STRINGS.
%
%   problemFile      - A string representing the .h5 file containing the problems. 
%
%   problem_indicies - (optional) An array of indices to use in the comparison.  
%                      If not specified, the entire problemFile will be used.
%
% OUTPUT:
%   results - A struct containing the results of comparison.  
solverHandles = {       
                 %'MLCPproxPGS'
                 'NCPproxPGS'
                 };
%problemFile =  'sim_data_13_04_19_19_44_03/SpheresFall.h5';
problemFile =  'sim_data_13_04_19_19_55_19/SpheresFall.h5';
                

%problemFile = 'logs.h5';
%problem_indicies = (18:23);
problem_indicies= (5 : 8);
Rvalue = 0.7:0.05:1;
%Rvalue = 1 : 0.1 : 20;
results = Compare_R(solverHandles, problemFile,  Rvalue, problem_indicies); 