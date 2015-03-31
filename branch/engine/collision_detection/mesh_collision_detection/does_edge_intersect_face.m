
% This file was originaly TriangleRayIntersection.m, and is adapted from 
% Jaroslaw Tuszynski's code at 
% http://www.mathworks.com/matlabcentral/fileexchange/33073-triangleray-intersection/content/html/TriangleRayIntersection_tutorial.html

% Copyright (c) 2011, Jaroslaw Tuszynski
% All rights reserved.
% 
% Redistribution and use in source and binary forms, with or without 
% modification, are permitted provided that the following conditions are 
% met:
% 
%     * Redistributions of source code must retain the above copyright 
%       notice, this list of conditions and the following disclaimer.
%     * Redistributions in binary form must reproduce the above copyright 
%       notice, this list of conditions and the following disclaimer in 
%       the documentation and/or other materials provided with the distribution
%       
% THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
% AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
% IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
% ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE 
% LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
% CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
% SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
% INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
% CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
% ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
% POSSIBILITY OF SUCH DAMAGE.


% This tests specifically for segment-TRIANGLE intersection.  
% All inputs should be row vectors
function intersect = does_edge_intersect_face(orig, dir, vert0, vert1, vert2)

    eps = 1e-5;
    intersect = false(size(orig,1),1);
    t = zeros(size(orig,1),1); u=t; v=t;

    edge1 = vert1-vert0;          % find vectors for two edges sharing vert0
    edge2 = vert2-vert0;
    tvec  = orig -vert0;          % distance from vert0 to ray origin
    pvec  = cross(dir, edge2,2);  % begin calculating determinant - also used to calculate U parameter
    det   = sum(edge1.*pvec,2);   % determinant of the matrix M = dot(edge1,pvec)
    parallel = (abs(det)<eps);    % if determinant is near zero then ray lies in the plane of the triangle
    if all(parallel), return; end % if all parallel then no intersections
%    switch border
%      case 'normal'
        zero=0.0;
%      case 'inclusive'
%        zero=eps;
%      case 'exclusive'
%        zero=-eps;
%    end
  
    det(parallel) = 1;                        % change to avoid division by zero
    u  = sum(tvec.*pvec,2)./det;              % calculate U parameter used to test bounds
    ok = (~parallel & u>=-zero & u<=1.0+zero);% mask which allows performing next 2 operations only when needed
    if ~any(ok), return; end                  % if all ray/plane intersections are outside the triangle than no intersections 
    qvec = cross(tvec(ok,:), edge1(ok,:),2);  % prepare to test V parameter
    v(ok,:) = sum(dir(ok,:).*qvec,2) ./ det(ok,:);  % calculate V parameter used to test bounds
    intersect = (v>=-zero & u+v<=1.0+zero & ok);  
    t(ok,:) = sum(edge2(ok,:).*qvec,2)./det(ok,:);
    intersect = (intersect & t>=-zero & t<=1.0+zero);

end



