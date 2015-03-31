
% Return the quaternion multiplication of q and r.
function qout = quatmultiply(q,r)
    if size(q,2)~=4 || size(r,2)~=4, error('q and r must be 1-by-4'); end

    % Calculate vector portion of quaternion product
    % vec = s1*v2 + s2*v1 + cross(v1,v2)
    vec = [q(:,1).*r(:,2) q(:,1).*r(:,3) q(:,1).*r(:,4)] + ...
          [r(:,1).*q(:,2) r(:,1).*q(:,3) r(:,1).*q(:,4)] + ...
          [q(:,3).*r(:,4)-q(:,4).*r(:,3) ...
             q(:,4).*r(:,2)-q(:,2).*r(:,4) ...
               q(:,2).*r(:,3)-q(:,3).*r(:,2)];

    % Calculate scalar portion of quaternion product
    % scalar = s1*s2 - dot(v1,v2)
    scalar = q(:,1).*r(:,1) - q(:,2).*r(:,2) - ...
               q(:,3).*r(:,3) - q(:,4).*r(:,4);

    qout = [scalar  vec];
end       
