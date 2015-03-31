%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
% mesh_ground.m 
%
% Generates a ground surface for little poly meshes to play on
 
function mg = mesh_ground(W, L, N, z)
 
%% (W) width, (L) length, (N) number of cells on a side
%% optional -- (ST) surface type
 
if (3 == nargin)
   z = 0;
end

% z is the height of the four corners of the ground: [BL; BR; TR; TL]

% We can't do a surface of random hills, unless we decompose it
% into many convex blocks.
 
H = 0.02; % height of surface

 
 
% create the bottom face
VERTS = [mesh_vertex([-W/2; -L/2; -H], []); ...
         mesh_vertex([W/2; -L/2; -H], []); ...
         mesh_vertex([W/2; L/2; -H], []); ...
         mesh_vertex([-W/2; L/2; -H], [])];
          
FACES = mesh_face([4 3 2 1]);
 

% create the top face
for incr_i = 1:N+1
    for incr_j = 1:N+1
   
        % calculate the elevation of this vertex
        this_X = (incr_j - 1) / N;
        this_Y = -(incr_i - 1) / N;
        this_Z = this_X * z;
        
        % create a vertex
        this_V = [-W/2 + this_X * W; ...
                   L/2 + this_Y * L; ...
                  this_Z];
        VERTS = [VERTS; mesh_vertex(this_V, [])]; 
        
    
    
        if (incr_i <= N && incr_j <= N)
            % create a face 
            
            this_F = [incr_i * (N + 1) + incr_j + 4, ...
                      incr_i * (N + 1) + incr_j + 5, ...
                      (incr_i - 1) * (N + 1) + incr_j + 5, ...
                      (incr_i - 1) * (N + 1) + incr_j + 4];
            FACES = [FACES; mesh_face(this_F)];
            
        end
            
        
    end
end



% Yes, there are some weird offsets going on in here


% create faces for the sides of the ground
% (L/R/B/T in relation to looking down in the -Z direction)
Vlist_top = zeros(1, N+3);
Vlist_left = zeros(1, N+3);
Vlist_right = zeros(1, N+3);
Vlist_bottom = zeros(1, N+3);

Vlist_top(1:2) = [3, 4];
Vlist_left(1:2) = [4, 1];
Vlist_right(1:2) = [2, 3];
Vlist_bottom(1:2) = [1, 2];

for incr_i = 1:N+1
    Vlist_top(incr_i + 2) = incr_i + 4;
    Vlist_left(incr_i + 2) = ((N+2) - incr_i - 1) * (N + 1) + 5;
    Vlist_right(incr_i + 2) = incr_i * (N + 1) + 4;
    Vlist_bottom(incr_i + 2) = N * N  + 2 * N - incr_i + 6;
end
    


    
    
FACES = [FACES; mesh_face(Vlist_top); ...
                mesh_face(Vlist_left); ...
                mesh_face(Vlist_right); ...
                mesh_face(Vlist_bottom)];

 
 
% populate face vector for all vertices
for incr_i = 1:length(FACES)
    for incr_j = 1:length(FACES(incr_i).verts)
        v = FACES(incr_i).verts(incr_j);
        VERTS(v).faces = [VERTS(v).faces; incr_i];
    end
end
 
 
 
%% create the mesh object
mg = obj_mesh(VERTS, FACES);
 
%% set attributes and stuff
mg.static = 1;
mg.J = eye(3); % I for the moment of inertia, since it is static
 
mg.color = [0.7 0.7 1.0];
mg.faceAlpha = 1.0;
mg.u = [0; 0; 0];
 
 
end
 
 
