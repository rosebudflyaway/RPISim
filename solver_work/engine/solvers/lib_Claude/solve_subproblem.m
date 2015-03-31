%
% solve_subproblem.m
% 
% Copyright (2012)
% Author: Claude Lacoursiere, 
% UMIT/HPC2N
% Umeå University, 
% 901 87 Umeå, 
% Sweden
% claude@hpc2n.umu.se
%
function   z = solve_subproblem(M, q, l, u, idx)     
  % usage: z = solve_subproblem(q, M, l, u, idx)
  %
  %  Solve the linear subproblem M(i, i)\z(i) = -q(i) - M(i, j)*z(j)
  %  where i is the list of active variables and j is the complement of
  %  i in 1:n, i.e., the list of inactive variables. 
  %  INPUTS:
  %  M     : a square matrix
  %  q     : a compatible vector
  %  l     : vector of lower bounds
  %  u     : vector of upper bounds
  %  idx   : current index set
  %  OUTPUTS:
  %  z     : the solution vector
  
  
  [i, j, jl, ju ] = split_indicies(idx) ; 
  z  = 0*q;
  z(jl) = l(jl); 
  z(ju) = u(ju); 
  % solve principal subproblem here
  if ~isempty(i)  
    if (~issparse(M))
      fprintf('solve_subproblem: matrix is not sparse!\n')
      M = sparse(M); 
    end%if
    z(i) = -M(i,i) \ (q( i) +  M(i, j)*z(j) );  %(A\b is actually inv(A)*b)
  end%if
end%function

