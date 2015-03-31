%
% block_switch.m
% 
% Copyright (2012)
% Author: Claude Lacoursiere, 
% UMIT/HPC2N
% Umeå University, 
% 901 87 Umeå, 
% Sweden
% claude@hpc2n.umu.se
%

function [idx, changes] = block_switch(idx0, z, l, u, w, tol)
         
  % usage:  idx = block_switch(idx0, z, l, u, w, tol)
  % IPUTS:
  % idx0:   current index set
  % z   :   candidate solution
  % w   :   candidate slack
  % l   :   lower bounds
  % u   :   upper bounds
  % tol :   error tolerance
  % 

  %  Index definition:
  EQUALITY = -1;
  FREE     =  0;
  LOWER    =  2;
  UPPER    =  4;
  
  idx = idx0;
  % This rule will do:
  % 1  : keep all active variables within bounds active
  % 2  : remove active variables which are outside the bounds
  % 3  : activate variables whose residuals have the wrong sign

  % The AND operator is done with multiplication .*
  % This could be done differently. 

  % EQUALITY: leave alone
  idx = EQUALITY * ( idx0 == EQUALITY );
  % Free variables with busted lower bound reset to LOWER
  idx = idx + ( idx0 == FREE  ) .* ( LOWER * ( z  < ( l - tol ) ) );
  % Free variables with busted upper bound reset to UPPER
  idx = idx +  ( idx0 == FREE  ).*  ( UPPER * ( z  > ( u + tol ) ) );
  % Variables at lower bound with positive residual stay LOWER
  % but otherwise switch to FREE
  idx = idx + LOWER * ( idx0 ==  LOWER ) .* ( w  >-tol ) ; 
  % Variables at upper bound with negative residual stay UPPER
  % but otherwise switch to FREE
  idx = idx + UPPER * ( idx0 ==  UPPER ) .* ( w  < tol ) ; 
      
  changes = sum(idx0~=idx);

end%function
