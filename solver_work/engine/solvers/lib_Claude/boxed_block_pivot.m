%
% boxed_block_pivot.m
% 
% 
% Copyright (2012)
% Author: Claude Lacoursiere, 
% UMIT/HPC2N
% Umeå University, 
% 901 87 Umeå, 
% Sweden
% claude@hpc2n.umu.se
function [ z, w, idx, it, err, errv] =  boxed_block_pivot(M, q, l, u, maxit, tol, idx, verbose)
  % usage:  [z, w, idx, it, errv] =  boxed_block_pivot(M, q, l, maxit, tol, idx, verbose )
  %
  % Block pivot algorithm for solving the mixed LCP defined as
  %  M * z + q = wplus - wminus
  %  min(z-l, wplus) == 0 && min(u-z, wminus) ==0
  % with wplus = max(w, 0); wminus = -min(w, 0);
  %
  % INPUTS:
  % M       :   square P matrix
  % q       :   vector with matching dimensions
  % l       :   vector of lower bounds, including -Inf
  % u       :   vector of upper bounds, including  Inf
  % maxit   :   maximum number of iterations
  % tol     :   numerical tolerance (optional)
  % idx     :   index set for warm start
  % verbose :   diagnostic output
  
  % OUTPUTS :
  % z       :   solution vector or best approximation within maxit
  % w       :   slack vector
  % idx     :   index set (see below for description of values)
  % it      :   negative on failure  (actually from code below, this is the number of iterations, and when negative, fail !!! )
  % errv    :   errors as a function of iteration
  %
  M = sparse(M); 

  % the index set is defined as follows:
  EQUALITY = -1;
  FREE     =  0;
  LOWER    =  2;
  UPPER    =  4;

  % argument processing
  if ( ~exist('maxit', 'var') )
    maxit = n ; 
  end%if
  if ( ~ exist('tol', 'var') )
    tol = 1E-10;
  end%if
  if ( ~exist('idx', 'var') ) 
    idx = FREE * ones(size(q));	% all variables start free
  end%if
  if ( ~exist('verbose', 'var') )
    verbose = 0;   % verbose is the diagnostic output
  end%if
  
  % initialization
  errv = [];
  it         = 0;    % This is the number of iterations 
  err        = 1;
  ix_list    = zeros(length(idx), 1);
      
  
  % find unbounded variables and leave them out of the switches
  idx(l==-Inf & u == Inf) = EQUALITY;    

  % solution loop.  Yup, it's that simple.
  % should actually just check that the number of changes is nil(0)
  done = 0;
  found = 0;
  cycles = 0; 
  
  while (~done)
    it = it + 1;
    z  = solve_subproblem(M, q, l, u, idx);
    w = M*z + q;
    w(idx<=FREE) = 0;
    [err] = get_complementarity_error(z, w, l, u);
    errv = [errv; err]; 
    [idx, changes] = block_switch(idx, z, l, u, w, tol);
    if (changes)
      [ found, ix_list, clength] =  cycle_detection(ix_list, idx);
      if (found)
	cycles = cycles + 1;
	[ix_list, idx]  = cycle_resolution(ix_list, idx, l, u, verbose );
      end%if
    end%if
    done = (err < tol || it >= maxit || ~changes); 
  end %while
  
  if ( cycles && it < maxit  && verbose )
    fprintf('Found %d cycles but succeded with %d iterations\n', cycles, it)
  end%if
  if ( cycles && it >= maxit && verbose )
    fprintf('Found %d cycles and failed at %d iterations\n', cycles, it)
  end%if
    
  % mark failures
  if (it >= maxit )
    [ee, r] = min(errv);
    idx = ix_list(:, r); 
    z  = solve_subproblem(M, q, l, u, idx);
    w  = M*z + q; 
    it = -it;
  end %if
     
end%function

