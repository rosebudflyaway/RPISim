%% CASE1: transform the nd friction magnitude into x, y direction 
%         transform the nd tangential velocity into x, y direction
%         compute the error. This is to transform the result into NCP form 
%         and compute the NCP errors.
%  INPUT: 
%        nc --------------------- number of contact
%        nd --------------------- number of facets in the LCP polyhedron
%        pf --------------------- frictional force along each of the nd direction 
%                                 a vector of length nc*nd, element >= 0
%        relVel ----------------- relative tangential velocity along each
%                                 of the nd direction, decided by the Gf  
%  OUTPUT: 
%         friction_error -------- is the total friction error normalized by
%                                 the number of contact

function [friction_error, pfMag] = updateFrictionError(nc, nd, pf, relVel)
pfMag = zeros(nc , 1);
VfMag = zeros(nc, 1);
alpha = zeros(nc, 1);
FrictionError = zeros(nc, 1);
 
    resultant_f  = zeros(nc, 2);
    resultant_vel = zeros(nc, 2);
    % alpha is the angle between Pf and (-Vf) for each contact
    theta = 2 * pi/nd;
    for i = 1 : nc
        for j = 1 : nd
            resultant_f(i, 1) = resultant_f(i, 1) + pf((i-1)*nd+j, 1) * cos((j-1)*theta);
            resultant_f(i, 2) = resultant_f(i, 2) + pf((i-1)*nd+j, 1) * sin((j-1)*theta);
            resultant_vel(i, 1) = resultant_vel(i, 1) + relVel((i-1)*nd+j, 1) * cos((j-1)*theta);
            resultant_vel(i, 2) = resultant_vel(i, 2) + relVel((i-1)*nd+j, 1) * sin((j-1)*theta);
        end
        
        % HERE TO DECIDE THE ANGLE, WE NEED VECTOR-3 FORM, ADD 0 TO F, vel
        Current_f = [resultant_f(i, :), 0];
        Current_vel = [resultant_vel(i, :), 0];
        % atan2 works better than acos for solving angles in [0, pi]
        % BE SUER IT IS (-Vf);
        % angle = atan2(norm(cross(a,b)),dot(a,b)) returns the angle in range [0, pi]
        alpha(i, 1) =  atan2(norm(cross(Current_f, -Current_vel)),dot(Current_f, -Current_vel)) ;
        pfMag(i, 1) = norm(Current_f);
        VfMag(i, 1) = norm(Current_vel);
        FrictionError(i, 1) = pfMag(i, 1) * VfMag(i, 1) * alpha(i, 1);
    end
friction_error = sum(FrictionError) / nc;
end