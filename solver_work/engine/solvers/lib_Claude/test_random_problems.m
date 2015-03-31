% 
% Copyright (2012)
% Author: Claude Lacoursiere, 
% UMIT/HPC2N
% Umeå University, 
% 901 87 Umeå, 
% Sweden
% claude@hpc2n.umu.se
%
function [problems, results] = test_random_problems(t, problem_size) % 100 250
  n          = problem_size;
  rk         = n; 			% rank 
  % NOTE: solvers quickly go bad when the condition number is large. 
  cond       = 1E4;			% condition number
  symm       = 1; 			% symmetric
  sp         = 0.10;			% sparse density
  degenerate = 0;
  free       = min(ceil(1+rand*n/2), n); % free variables
  tlower     = min(ceil(1+rand*n/3), n); % tight upper bound variables
  % things can also get bad when the range for the bounds is too small.
  %  This is related to the condition number
  range      = 1000;
  max_iterations = 25;
  tolerance  = 1E-8;

  solvers = { struct('name', 'Block pivot', 'func', @boxed_block_pivot), ...
	     struct('name', 'Zhang Gao', 'func', @zhang_gao_smoothing), ...
	     struct('name', 'Li Fukushima', 'func', @li_fukushima_newton)
	     };
  
  result  = struct('idx', [], 'z', [], 'err', [], 'timing', [], 'it', []);
  results ={result, result, result};
  
  for i= 1:t
    fprintf('Working on problem %d... \n', i);
    [M, z0, q, w0, idx0, l, u] = rand_blcp_chen(n, rk, cond, sp, degenerate, free,...
						tlower,range, range, range);
    problems{i} = struct('idx', idx0, 'z', z0); 

    for s = 1:length(solvers)
      tic;
      [z, w, idx, it, err] = solvers{s}.func(M, q, l, u, max_iterations, tolerance);
      tt = toc();
      results{s}.name = solvers{s}.name;
      results{s}.z      = [results{s}.z; z'];
      results{s}.err    = [results{s}.err; norm(z-z0)];
      results{s}.idx    = [results{s}.idx; idx'];
      results{s}.timing = [results{s}.timing; tt];
      results{s}.it     = [results{s}.it; it];
    end%for
  end%for
end%function
