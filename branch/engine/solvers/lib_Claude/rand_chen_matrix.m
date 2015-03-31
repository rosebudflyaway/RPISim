% 
% Copyright (2012)
% Author: Claude Lacoursiere, 
% UMIT/HPC2N
% Umeå University, 
% 901 87 Umeå, 
% Sweden
% claude@hpc2n.umu.se
%

function M = rand_chen_matrix (n, gamma, tau, sp)
  
  % usage:  M = rand_chen_matrix (n, gamma, tau, sp)
  %
  % Generate a random, symmetric positive definite square matrix with
  % known condition number, and approximate fill density.
  %
  % The distribution of eigenvalues is uniform on a logarithmic scale
  % between sqrt(1/tau) and sqrt(tau).
  %
  % INPUTS:
  % n     : the order of the matrix 
  % gamma : the rank
  % tau   : the condition number.  If the matrix is rank deficient,
  %         this is then the ratio of the biggest to the smallest non-zero
  %         eigenvalue
  % sp    : is the approximate sparse fraction
  % OUTPUTS:
  % M     : the matrix
  % 
  % first off, generate diagonal matrix with eigenvalues in the given range
  sigma           = zeros(n, 1) ; 	 %init diagonal matrix
  tau             = sqrt(tau);		 
  sigma(1,1)      = 1/tau ; 
  sigma(gamma,1)  = tau; 

  % Diagonal elements with index above gamma are kept to zero so the
  % matrix is degenerate with gamma zero eigenvalues.

  % Non-zero diagonal elements are set so the condition number is
  % respected.  The first non-zero element is set to 1/sqrt(tau), the last to
  % sqrt(tau), the ones in between are randomly distributed to be between
  % these values so that the max eigenvalue over the smallest one is
  % tau^ 2.  
  for k = 2:(gamma - 1) 
    nu         = 1 - 2*rand();			
    sigma(k,1) = tau^nu ; 
  end%for


  M    = sparse(diag(sigma));
  Q = rand_ortho(n, sp);
  M = M*Q; 
  M = Q' * M ; 
  
end%function 
