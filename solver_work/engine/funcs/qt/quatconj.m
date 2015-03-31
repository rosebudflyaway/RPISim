
% Returns the conjugate of quaternion qin
function qout = quatconj( q ) 
  if (size(q,2) ~= 4), error(message('q should be 1-by-4')); end
  qout = [ q(:,1)  -q(:,2:4) ];
end
