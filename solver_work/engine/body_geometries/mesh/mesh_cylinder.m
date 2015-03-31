%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
% mesh_cylinder.m 
%
% Generates a cylinder mesh object


function cyl = mesh_cylinder(num_verts_per_end, m, r, h)
    % (m) mass, (r) radius, (h) height

    % Read in poly file
%      cyl = mesh_read_poly_file('cylinder.poly'); 

%      num_verts_per_end = 12;

    VERTS = [];
    FACES = [];
    
    % Generate vertices and faces:
    
    % (top end)
    VERTS = mesh_vertex([0; 0; 0.5], []); % center point
    center_v = 1;
    this_v = 1;
    
    for incr_v = 1:num_verts_per_end
        th = ((incr_v - 1) / num_verts_per_end) * 2 * pi;
        VERTS = [VERTS; mesh_vertex([cos(th); sin(th); 0.5], [])];
        this_v = this_v + 1;
        
        if (incr_v > 1)
            last_v = this_v - 1;
            bottom_v = 4 + 2 * num_verts_per_end - incr_v;    
        else
            last_v = num_verts_per_end + 1;
            bottom_v = 3 + num_verts_per_end;
        end

        F_end = [center_v; last_v; this_v];
        F_side = [this_v; last_v; bottom_v];
        
        FACES = [FACES; mesh_face(F_end); mesh_face(F_side)];
    end

    % (bottom end)
    VERTS = [VERTS; mesh_vertex([0; 0; -0.5], [])]; % center point
    center_v = num_verts_per_end + 2;
    this_v = this_v + 1;
    
    for incr_v = 1:num_verts_per_end
        th = ((num_verts_per_end - incr_v + 1) / num_verts_per_end) * 2 * pi;
        VERTS = [VERTS; mesh_vertex([cos(th); sin(th); -0.5], [])];
        this_v = this_v + 1;
    
        if (incr_v < num_verts_per_end)
            next_v = this_v + 1;
            top_v = num_verts_per_end - incr_v + 2;
        else
            next_v = num_verts_per_end + 3;
            top_v = 2;
        end
        
        F_end = [center_v; this_v; next_v];
        F_side = [top_v; next_v; this_v];
        
        FACES = [FACES; mesh_face(F_end); mesh_face(F_side)];
    end

   cyl = bodyMesh(VERTS, FACES);
    
   % Moment of inertia
   cyl.J = diag([m*(3*r^2 + h^2)/12, m*(3*r^2 + h^2)/12, (m*r^2)/2]);
   cyl.Jinv = inv(cyl.J); 
    
   % Scale the cylinder for (r) width and (h) height  
   for incr_v = 1:cyl.num_verts
      cyl.verts(incr_v).local_coords(1:2) = r * cyl.verts(incr_v).local_coords(1:2);
      cyl.verts(incr_v).local_coords(3) = h * cyl.verts(incr_v).local_coords(3);
   end
    
end

