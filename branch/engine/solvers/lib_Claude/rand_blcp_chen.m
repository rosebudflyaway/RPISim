% 
% Copyright (2012)
% Author: Claude Lacoursiere, 
% UMIT/HPC2N
% Umeå University, 
% 901 87 Umeå, 
% Sweden
% claude@hpc2n.umu.se
%


function [ M, z, q, w, idx, l, u] =  rand_blcp_chen(n, gamma, tau, ...
						  sp, n1, n2, n3,  m1, m2, m3 )

  %
  %
  % [M, z, q, idx] = rand_blcp_chen(n, gamma, tau, symmetric, sp, n1, n2, m1, m2) 
  % Generate a random boxed LCP with known solution.
  % 
  % The LCP definition is   M*z + q = w_plus - w_minus with
  % 0 =< z-l   perp  w_plus >= 0
  % 0 =< u-z   perp  w_minus >= 0
  % INPUTS
  % n      : is the dimension of the matrix
  % gamma  : is the rank of the matrix
  % tau    : is the condition number of the matrix,
  %          excluding eventual 0 eigenvalues in case of rank degeneracy. 
  % sp     : approximate fill fraction of the matrix
  % n1     : number of degenerate complementarity points
  % n2     : number free variables in solution. 
  % n3     : number tight variables at upper bound
  %  NOTE: the left over variables are thight at lower bound ( n - n1 - n2 -n3) ;
  % m1     : is the numerical range for the slack variables,
  %          distributed uniformly in [-m1, m1]
  % m2     : numerical range of tight variables
  % m3     : numerical range of the box bounds.
  %          NOTE: the box bounds are symmetric so that upper = -lower 
  %          and upper, lower are uniformly distributed in [-m3, m3]
  % OUTPUTS: 
  %
  % M      : an n x n sparse matrix with given properties
  % q      : n dimensional q vector  
  % z      : n dimensional solution vector 
  % w      : n dimensional slack variables
  % idx    : the index set: idx==-1 are the equality variables
  %          idx==0 are the free variables, idx==2 are the
  %          variable tight at 0, idx==4 are the variables tight at
  %          upper bound
  % l      : lower bounds
  % u      : upper bounds
  % 
  EQUALITY = -1;
  FREE     =  0;
  LOWER    =  2;
  UPPER    =  4;
  M  = rand_chen_matrix(n, gamma, tau, sp) ; 
  % now, generate the lcp variables.
  if ( n1 > n ) 
    disp('rand_blcp_chen : wrong parameters: n1 must be less than n '); 
    return
  elseif ( (n1+n2) > n ) 
    disp('rand_blcp_chen : wrong parameters: n1 + n2 must be less than n '); 
    return
  elseif ( (n1+n2+n3) > n ) 
    disp('rand_blcp_chen : wrong parameters: n1 + n2 + n3 must be less than n '); 
    return
  end%if

  l = -1 - m3*rand(n, 1);
  u =  1 + m3*rand(n, 1);

  idx = zeros(n, 1); 

  n1 = ceil(n1);
  n2 = ceil(n2);
  n3 = ceil(n3);
  % set the free variables
  idx((n1 + 1):(n1 + n2), :) = FREE * ones(n2, 1); 

  % set the variables at lower bounds
  if ( n1 + n2 < n ) 
    idx((n1 + n2 + 1):n, :) = LOWER*ones(n - n2 - n1, 1); 
  end%if
  % set the variables at upper bounds
  if ( n1 + n2 + n3 < n ) 
    idx((n1 + n2 + n3 + 1):n, :) = UPPER*ones(n - n1 - n2 - n3, 1); 
  end%if
  idx = idx(randperm(n)); % randperm(n) returns a row vector containing a random permutation of the integers from 1 to n inclusive
  
  z = zeros(n, 1); 
  q = zeros(n, 1); 
  
  j0 = find(idx==FREE); 
  j2 = find(idx==LOWER); 
  j4 = find(idx==UPPER); 
  z(j0)  = l(j0) + (u(j0)- l(j0)).*rand(size(j0));
  z(j2)  = l(j2);
  z(j4)  = u(j4);
  q(j2)  =  m2*rand(size(j2));
  q(j4)  = -m2*rand(size(j4));
  q = q - M*z; 

  w = M*z + q ; 
  end %function
 