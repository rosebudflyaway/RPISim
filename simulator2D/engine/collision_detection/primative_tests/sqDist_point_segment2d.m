

% Given points a,b, and c, returns the square distance from c to the segment
% defined by (a -> b).
%
%   For example:
%                 b *
%         c *      /
%                 /  
%                /
%             a *

function sqD = sqDist_point_segment2d( a,b,c )

    ab = b-a;
    ac = c-a; 
    bc = c-b;
    e = dot(ac,ab);
    % First case where c projects outside ab
    if e <= 0
        sqD = dot(ac,ac);
        return;
    end
    f = dot(ab, ab);
    % Second case where c projects outside ab
    if e >= f
        sqD = dot(bc,bc);
        return;
    end
    % Case where c projects onto ab
    sqD = dot(ac,ac) - e*e / f;

end

