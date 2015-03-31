% 
% Copyright (2012)
% Author: Claude Lacoursiere, 
% UMIT/HPC2N
% Umeå University, 
% 901 87 Umeå, 
% Sweden
% claude@hpc2n.umu.se
%

% very simple test over 100 problems of size 250.  The condition number
% of these problems is set the in the test_random_problems scripts and
% affects the algorithms very strongly, so are the bounds allowed for
% the variables.  See the script for details. 

[p, r] = test_random_problems(100, 250); % p here is the problems about the initial value
comparison_plots(r{1}, r{2}, r{3});
