

% Given a polygonal P = [ x1 y1 
%                          ...
%                         xn yn ], 
% and the grid parameters, cd_polygon_grid returns a boolean matrix where 1
% represents collision of P with that grid cell and 0 represents
% non-collision.  

function collision_grid = cd_polygon_grid( P, grid_center, grid_Rot, grid_minX, grid_maxX, grid_minY, grid_maxY, grid_dx, grid_dy )

    m = floor( (grid_maxX - grid_minX)/grid_dx );
    n = floor( (grid_maxY - grid_minY)/grid_dy );

    collision_grid(m,n) = false;
    
    dY = grid_Rot*[0;grid_dx];
    dX = grid_Rot*[grid_dy;0]; 

    bottom_left = grid_center + grid_Rot * [grid_minX; grid_minY];   % Start in bottom left corner 
    for i = 1:m                             % Loop over grid cells
       p0 = bottom_left + (i-1) * dY;
       for  j = 1:n
           collision_grid(m-i+1,j) = cd_polygon_polygon( [p0 p0+dX p0+dX+dY p0+dY]' , P );
           p0 = p0 + dX; 
       end
    end

end

