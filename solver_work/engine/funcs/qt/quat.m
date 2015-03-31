
% Returns a 1x4 vector representing a quaternion
% rotation of theta radians about vector k.
% Note: the scalar is the first element q(1). 
function q = quat( k,theta )
  k = k/norm(k);        %normalize k
  if size(k,1) > 1
    q = [cos(theta/2) k'*sin(theta/2)];
  else
    q = [cos(theta/2) k*sin(theta/2)];
  end
  
