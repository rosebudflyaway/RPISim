%
% split_indicies.m
%
% Copyright (2012)
% Author: Claude Lacoursiere, 
% UMIT/HPC2N
% Umeå University, 
% 901 87 Umeå, 
% Sweden
% claude@hpc2n.umu.se
function [i, j, jl, ju, eq ] = split_indicies(idx)

  % usage:  XXX = split_indicies (XXX)
  %
  % Given an index set idx, extract the set of variables which satisfy
  % the following conditions:

  EQUALITY  = -1; % equality constraints always in the active set
  FREE      =  0; % variables that are currently within bounds
  LOWER     =  2; % variables clamped at lower bound
  UPPER     =  4; % variabled clamped at upper bound

  % INPUTS:
  % idx   : an index set
  % OUTPUTS:
  % i      : indicies of active variables
  % j      : indicies of variables set at bounds
  % jl     : subset of j with variables at lower bounds
  % ju     : subset of j with variables at upper bounds
  % eq     : equality variables, those which are always active

  % NOTA: should have another category to include variables set at a
  % bound which should never be switched.  

  eq = find(idx == EQUALITY); 
  i  = find(idx <= FREE    ); 
  j  = find(idx >  FREE    ); 
  jl = find(idx == LOWER   ); 
  ju = find(idx == UPPER   ); 

end%function
