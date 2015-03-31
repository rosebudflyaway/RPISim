% 
% Copyright (2012)
% Author: Claude Lacoursiere, 
% UMIT/HPC2N
% Umeå University, 
% 901 87 Umeå, 
% Sweden
% claude@hpc2n.umu.se
%

function [ M , z, q , idx] =  rand_lcp_chen(n, gamma, tau, sp, n1, n2, m1, m2 ) 
  %
  %
  % [M, z, q, idx] = rand_lcp_chen(n, gamma, tau,  sp, n1, n2, m1, m2) 
  % 
  % Generate a random LCP with known solution.
  % The LCP definition is  0 =<  M*z + q = w  perp  z >= 0
  % INPUTS
  % n      : is the dimension of the matrix
  % gamma  : is the rank of the matrix
  % tau    : is the condition number of the matrix,
  %          excluding eventual 0 eigenvalues in case of rank degeneracy. 
  % sp     : approximate fill fraction of the matrix
  % n1     : number of degenerate complementarity points
  % n2     : number free variables in solution. 
  %  NOTE: the left over variables are thight ( n - n1 - n2 ) ;
  % m1     : is the numerical range for the slack variables,
  %          distributed uniformly in [-m1, m1]
  % m2     : numerical range of tight variables
  % OUTPUTS: 
  %
  % M      : an n x n sparse matrix with given properties
  % q      : n dimensional q vector  
  % z      : n dimensional solution vector 
  % w      : n dimensional slack variables
  % idx      : the index set: idx==0 are the free variables, idx==2 are the
  %          variable tight at 0

  M  = rand_chen_matrix(n, gamma, tau, sp) ; 


				% now, generate the lcp variables.
  if ( n1 > n ) 
    disp('rand_lcp_chen : wrong parameters: n1 + n2 must be less than n '); 
    return
  elseif ( (n1+n2) > n ) 
    disp('rand_lcp_chen : wrong parameters: n1 + n2 must be less than n '); 
    return
  end%if

  % make place holders for degenerate points
  idx = -ones(n, 1); 

  idx((n1+1):(n1+n2), :) = zeros(n2, 1);
  if ( n2 + n1 < n ) 
    idx((n1+n2+1):n, :) = 2*ones(n-n2-n1, 1);
  end%if
  idx = idx(randperm(n));

  z = zeros(n, 1); 
  q = zeros(n, 1); 

  z(idx==0, 1)  = m1*rand(n2, 1) ; 
  q(idx==2, 1)  = m2*rand(n- n2 - n1, 1) ; 

  % reset degenerate indices

  %idx(find(J==-1)) = 0 ; 
  q = q - M*z; 
  w = M*z + q; 

endfunction




