% -*- octave -*-
% 
% Copyright (2012)
% Author: Claude Lacoursiere, 
% UMIT/HPC2N
% Umeå University, 
% 901 87 Umeå, 
% Sweden
% claude@hpc2n.umu.se

function [ found, ix_list, clength  ] =  cycle_detection(ix_list0, idx)
%
% given a an array I and an index set idx (just a list of integers),
% check that idx is not in I.  If not, just add it and return found=0.
% Otherwise, don't add it and return found = 1 
%
  ix_list = ix_list0;
  found = 0 ; 
  clength = Inf;
  for k = 1:size(ix_list, 2)
    if ~isempty(ix_list(:, k))
      found = isempty(find(idx ~= ix_list(:, k), 1)); 
    end%if
    if found
      found = k ; 
      clength = size(ix_list0, 2)+1 - k;
      break ; 
    end%if
  end%for
  
  if ~found 
    ix_list = [ix_list, idx]; 
  end%if
  
end%function

