%% CASE 2: COMPUTE THE ERROR OF FRICTION AND VELOCITY ALONG THE nd direction, then add the error together
%          This is the LCP error rather than NCP error
%          First get the vector of the frictional force along each of the
%          nd direction

%  INPUT:   
%  nc  --------------------------- number of contact 
%  nd  --------------------------- number of facets of polyhedron, or number of frictional direction
%  pf  --------------------------- nc*nd by 1 vector, frictional force,
%                                  each element is >= 0
%  relVel ------------------------ relative tangential velocity on the
%                                  fritional plane, decided by the Gf
function [friction_error, Discretize_pfMag] = updateDiscretizeFrictionError(nc, nd, pf, relVel)
    Discretize_Pf = zeros(nc*nd, 2);
    Discretize_Vf = zeros(nc*nd, 2);
    Discretize_alpha = zeros(nc*nd, 1);
    Discretize_FrictionError = zeros(nc*nd, 1);
    theta = 2 * pi/nd;
    for i = 1 : nc 
        for j = 1 : nd 
            Discretize_Pf((i-1)*nd + j, :) = pf((i-1)*nd+j, 1)*[cos((j-1)*theta), sin((j-1)*theta)];
            Discretize_Vf((i-1)*nd + j, :) = relVel((i-1)*nd+j, 1) * [cos((j-1)*theta), sin((j-1)*theta)];
        end
    end
    Discretize_pfMag = zeros(nc*nd, 1);
    Discretize_VfMag = zeros(nc*nd, 1);
    for i = 1 : nc*nd
        Discretize_pfMag(i, 1) = norm(Discretize_Pf(i, :));
        Discretize_VfMag(i, 1) = norm(Discretize_Vf(i, :));
        a = [Discretize_Pf(i, :), 0];
        b = [Discretize_Vf(i, :), 0];
        Discretize_alpha(i, 1) = atan2(norm(cross(a, -b)), dot(a, -b));
        Discretize_FrictionError(i, 1) = Discretize_pfMag(i, 1)*Discretize_VfMag(i, 1)*Discretize_alpha(i, 1);
    end
    friction_error = sum(Discretize_FrictionError) / (nc*nd);
end
 