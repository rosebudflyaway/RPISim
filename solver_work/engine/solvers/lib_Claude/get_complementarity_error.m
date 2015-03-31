% 
% Copyright (2012)
% Author: Claude Lacoursiere, 
% UMIT/HPC2N
% Umeå University, 
% 901 87 Umeå, 
% Sweden
% claude@hpc2n.umu.se
%


function [err, ee, idx] = get_complementarity_error (z, w, l, u, tol)

  % usage:  err = get_complementarity_error (z, w, l, u)
  %
  % Compute the complementarity error.
  % There are a few ways to do this.  First note that MLCP corresponds
  % to the two simultaneous conditions:
  %  min(z-l, max(w, 0))  == 0   and  min(u-z, max(-w,0)) ) == 0.  
  % The sum of the absolute values is therefore a good indicator.
  % However, the function
  % 2*z-l-u - abs(z-l-w) + abs(z-u-w)
  % also vanishes if and only if the mixed complementarity condition is
  % satisfied.  
  % This is described in the Li-Fukusima 2001 paper "Smoothing Newton
  % and Quasi-Newton Methods for Mixed Complementarity problems.  There
  % is less computation invovled here.
  % 
  ee = 0*z; 
  err = 0; 
  idx = -1; 
  %
  if (nargin > 4 )
    tol = 1E-10;		% yet another black magic number. 
  end%if
  % variables bounded below but not above
  infup = find(isinf(u) & isfinite(l));
  % variables above but not below
  inflow = find(isinf(l) & isfinite(u));
  % free variables
  free  = find(isinf(u) & isinf(l)); 
  %  variables bounded above and below
  box = find(isfinite(u) & isfinite(l));

  % standard complementarity error: min of w and z should be 0
  % otherwise, one of them is negative, or neither is 0
  ee(infup) = min(( z(infup)-l(infup) ),  w(infup) ) ;
  % inverted logic for the multipliers bounded above
  ee(inflow) = min (( u(inflow) - z(inflow) ) , -w(inflow) ) ; 
  ee(free) = (w(free));
  wplus  = max(w(box), 0);	% positive part
  wminus = - min(w(box), 0) ; 	% negative part
  ee(box) = abs( min ( z(box) - l(box), wplus ) ) + abs ( min ( u(box) - z(box),  wminus) ) ; 
  
  %
  % Get the list of the bad indicies
  % 
  if (~isempty(ee))
    err = norm(ee)/ sqrt(length(ee)); 
    if (err ~=0 )
      idx = find(ee~=0); 
    end%if
  end%if

end%function
