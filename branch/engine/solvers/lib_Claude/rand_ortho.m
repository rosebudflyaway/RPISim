% 
% Copyright (2012)
% Author: Claude Lacoursiere, 
% UMIT/HPC2N
% Umeå University, 
% 901 87 Umeå, 
% Sweden
% claude@hpc2n.umu.se
%


function Q = rand_ortho (n, f)
  
  % 
  % usage:  Q = rand_ortho (n, f)
  %
  % Generate an orthogonal sparse matrix of dimension n and fill
  % fraction approximately f.
  %
  % The matrix is tenerated using a number of Givens rotations on
  % randomly chosen rows and columns
  % 
  % INPUTS
  % n     : order of the square matrix
  % f     : approximate fill fraction
  % OUPUTS:
  % Q     : a sparse random orthogonal matrix


  Q     = speye(n);
  sq    = n*n;
  f     = f/2;
  maxct = n;
  count = 1;

  while ((nnz(Q)-n)/sq < f && count < maxct )
    t = pi*rand();
    c = cos(t); s=sin(t);
    g = [c, -s; s, c];
    i = ceil((n-1)*rand());
    j = i+ceil((n-i)*rand());
    Q(:, [i,j]) = Q(:, [i,j])*g;
    count = count + 1;
  end %while

end%function

