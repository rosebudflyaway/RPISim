%
% Given two polygons A and B of the form
%
% A = [ x1 y1
%       x2 y2
%        ...
%       xn yn ]
%
% where each vertex vi = [xi yi] and vertices are listed in
% counter-clockwise order around the polygon (B is similarly defined),
% cd_polygon_polygon returns true if the two polygons are interpenetrating
% and false otherwise.  cd_polygon_polygon usues the separating axis
% theorem to determine this interpentration.  Optimizations are possible
% that store the axis from the previous test and use this as the first axis
% during the new test.  
%

function intersecting = intersecting_poly_poly( A, B )

    R90 = [ 0 1; -1 0];  % 90 degree rotation matrix when multiplying p'R

    % Try the normals from A's edges as the separating axis
    % The first normal will be that of the edge between the last and first
    % vertices of A.  
    
    %% Use edges of A
    axs = (A(end,:) - A(1,:)) * R90;        % Determine axis
    axs = axs/norm(axs); 
    
    projectA = A*axs';                      % Project A onto axs
    minA = min(projectA);
    maxA = max(projectA);
    projectB = B*axs';                      % Project B onto axs
    minB = min(projectB);
    maxB = max(projectB);
    
    if minA < minB                          % Determine separation by axs
       if minB - maxA > 0  
           intersecting = false; return;
       end
    else
       if minA - maxB > 0
           intersecting = false; return;
       end
    end
    
    for i = 1:size(A,1)-1
        axs = (A(i,:) - A(i+1,:)) * R90;    % Determine axis
        axs = axs/norm(axs); 

        projectA = A*axs';                  % Project A onto axs
        minA = min(projectA);
        maxA = max(projectA);
        projectB = B*axs';                  % Project B onto axs
        minB = min(projectB);
        maxB = max(projectB);

        if minA < minB                      % Determine separation by axs
           if minB - maxA > 0  
               intersecting = false; return;
           end
        else
           if minA - maxB > 0
               intersecting = false; return;
           end
        end
    end
    
    %% Repeat using edges of B
    axs = (B(end,:) - B(1,:)) * R90;        % Determine axis
    axs = axs/norm(axs); 
    
    projectA = A*axs';                      % Project A onto axs
    minA = min(projectA);
    maxA = max(projectA);
    projectB = B*axs';                      % Project B onto axs
    minB = min(projectB);
    maxB = max(projectB);
    
    if minA < minB                          % Determine separation by axs
       if minB - maxA > 0  
           intersecting = false; return;
       end
    else
       if minA - maxB > 0
           intersecting = false; return;
       end
    end
    
    for i = 1:size(B,1)-1
        axs = (B(i,:) - B(i+1,:)) * R90;    % Determine axis
        axs = axs/norm(axs); 

        projectA = A*axs';                  % Project A onto axs
        minA = min(projectA);
        maxA = max(projectA);
        projectB = B*axs';                  % Project B onto axs
        minB = min(projectB);
        maxB = max(projectB);

        if minA < minB                      % Determine separation by axs
           if minB - maxA > 0  
               intersecting = false; return;
           end
        else
           if minA - maxB > 0
               intersecting = false; return;
           end
        end
    end

    intersecting = true;
end















