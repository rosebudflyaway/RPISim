%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
% test_CDA_broadphase.m
%
% Broadphase collision detection -- apply minimum distance epsilon, and trim
% all bodies further than minimum distance out of body structure

function [num_manifolds, num_subcontacts, B_out] = test_CDA_broadphase(B, V, cur_eps)

B_out = [];
num_bodies = length(B); % number of bodies

num_manifolds = 0; % number of manifolds
num_subcontacts = 0; % number of subcontacts


% If any edge on a body is within distance epsilon, then all edges of that body
% are considered in the formulation

% the problem is that the edges are not represented well in the B data structure,
% so trimming B.x and B.y will not produce the desired results

for incr_b = 1:num_bodies
    
    % Check the dimensions of B.x and B.y vertex vectors
    if (length(B(incr_b).x) ~= length(B(incr_b).y))
        error('Dimension mismatch for body vertex vectors');
    % go sit in the corner, and think about your life
    end
    
    % check each edge in this body
    d_less_than_eps = 0;
    for incr_v = 2:length(B(incr_b).x)
    
        % (pointSegmentDistance.m is in engine/funcs/)
        [sdx, ...
        sdy, ...
        sd_distance, ...
        sd_nv] = pointSegmentDistance(V.x, ...
                            V.y, ...
                            B(incr_b).x(incr_v - 1), ...
                            B(incr_b).y(incr_v - 1), ...
                            B(incr_b).x(incr_v), ...
                            B(incr_b).y(incr_v));
    
        if (sd_distance <= cur_eps)
            d_less_than_eps = 1;
        end
    
    % If uncommenting this, make sure to change incr_v = 1:length(B(incr_b).x) above
    %      if (norm([V.x - B(incr_b).x(incr_v); V.y - B(incr_b).y(incr_v)]) <= cur_eps)
    %        d_less_than_eps = 1;
    %      end
    end
    
    if (1 == d_less_than_eps)
        % add the body to B_out and increment num_bodies
        
        num_manifolds = num_manifolds + 1;
        if (1 == num_manifolds)
            % first one
            B_out = B(incr_b);
        else
            B_out(num_manifolds) = B(incr_b);
        end
        num_subcontacts(num_manifolds) = length(B_out(num_manifolds).x) - 1;
    
    end
    
end
    
    

