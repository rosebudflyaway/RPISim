% 
% Copyright (2012)
% Author: Claude Lacoursiere, 
% UMIT/HPC2N
% Umeå University, 
% 901 87 Umeå, 
% Sweden
% claude@hpc2n.umu.se
%

function [ix_list, idx] = cycle_resolution (ix_list0, idx0, l, u, verbose)

  % usage:  [ix_list, idx] = cycle_resolution (ix_list0, idx0, l, u, verbose)
  %
  % Resolve the collision in the list of index set by repeatedly
  % flipping the least wrong index until we have an unseen index set.
  % 
  % INPUTS:
  % ix_list0= list of all seen index set
  % idx0    = candidate index set
  % l       = lower bounds
  % u       = upper bounds
  % verbose = print diagnostics
  % OUTPUTS:
  % ix_list = index list including new index
  % idx     = candidate index set

  idx = idx0;
  ix_list = ix_list0;

  EQUALITY = -1;
  FREE     =  0;
  LOWER    =  2;
  UPPER    =  4;

  if ( exist('verbose', 'var') )
    verbose = 0;
  end%if
  done = 0;
  candidates = find(idx > EQUALITY  );
  position = 1; 
  restarts = 0;
  passes = 0;
  
  while ( ~done &&  position <= length(candidates) )
    passes = passes + 1;
    flip = candidates(position);
    if (idx(flip) == FREE && l(flip) > -Inf )
      idx(flip) = LOWER;
    elseif (idx(flip) == FREE && u(flip) < Inf )
      idx(flip) = UPPER;
    elseif (idx(flip) == LOWER )
      idx(flip) = FREE;
    elseif (idx(flip > FREE ))
      idx(flip) = FREE;
    end%if
    [found, ix_list, clength] = cycle_detection(ix_list, idx); 
    if ( ~found )
      done = 1; 
    elseif (found && position <  length(candidates))
      position = position + 1;
    else
      if ( verbose )
	fprintf('Restarting search %dth time\n', restarts);
     restarts = restarts + 1;
      end%if
      candidates = find(idx >= FREE );
      idx(candidates) = FREE;
      if ( verbose )
	   fprintf('Number of candidates = %d\n', length(candidates));
      end%if
      position = 1; 
    end%if
  end%while
  if (  ~done && verbose )
    fprintf('Cannot resolve index set conflict\n' );
  elseif ( verbose )
    fprintf('Resolved conflict in %d passes\n', passes);
  end%if
  end%function
