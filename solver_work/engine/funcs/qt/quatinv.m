% Returns the quaternion inverse of quaternion q.
function qinv = quatinv( q )
  qinv  = quatconj( q )./(sum(q.^2,2)*ones(1,4));
end

