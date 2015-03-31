% 
% Copyright (2012)
% Author: Claude Lacoursiere, 
% UMIT/HPC2N
% Umeå University, 
% 901 87 Umeå, 
% Sweden
% claude@hpc2n.umu.se
%

% -*- octave -*-
function [ s , w ] = get_complementarity_point(M, q, z, l, u, idx)
[i, j, jl, ju, eq] = split_indicies(idx);

w    = zeros(size(z));
s    = zeros(size(z));
if ~isempty(j)
  w(j) = M(j, :)*z + q(j) ;		% std formula
  s(jl) =  w(jl); 
  s(ju) = -w(ju); 
end 
if ~isempty(i)
  s(i) = min(z(i)-l(i), u(i)-z(i));	
end
if ~isempty(eq)
  s(eq) = 0 ;				% neglect equalities
end
end%function
