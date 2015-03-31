%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
% mesh_read_poly_file.m 
%
% Given an input file containing information on a mesh, the file is parsed
% and a 'bodyMesh' object is created representing that data. 
% 
% The input file should be of the form:
% ------------------------------------
% [number of vertices]
% 1 x1 y1 z1
% 2 x2 y2 z2
%   ...
% [number of faces]
% v1 v2 ... vn
% v1 v2 ... vn
% ...
% ------------------------------------
% 
% See cube.poly as an example.  

function mesh = mesh_read_poly_file( filename )

    num_verts = 0;
    num_faces = 0;
    
    file_contents = fileread(filename);    % Read in entire file (as cells) 
    poly_data = regexp(file_contents, '[^\n]*[^\n]*', 'match'); % Separate by line
    
    for i=1:length(poly_data)  % Remove stuff from poly_data
        poly_data(i) = regexprep(poly_data(i),'#(.*)','');    % Bash style comments
        poly_data(i) = regexprep(poly_data(i),'%(.*)','');    % MATLAB style comments
        poly_data(i) = regexprep(poly_data(i),'//(.*)','');   % C++ style comments
        poly_data(i) = regexprep(poly_data(i),'[\[\];=]',' '); % Remove MATLAB brackets and ';'s
        poly_data(i) = regexprep(poly_data(i),'[a-zA-Z]',''); % Remove characters, e.g. v and f from .obj files
    end
    poly_data(strcmp(regexprep(poly_data,'\s',''),'')) = []; % Remove empty entries 
    
    %% Here, we assume that poly_data is strictly data (no comments or meaningless lines)
    num_verts = str2num(char( poly_data(1) ));  % First line should be # of verts
    num_faces = str2num(char( poly_data(num_verts+2) )); 
    
    if length(poly_data) ~= num_verts+num_faces + 2
        error(['Problem reading mesh file (check face #s) ' filename]);   % Check data size
    end
    
    VERTS = [];
    FACES = [];
    
    % Populate vertices
    if length(str2num(char( poly_data(2) ))) == 4
        vIndexed = 1;
    else
        vIndexed = 0;
    end
    for i=2:num_verts+1
        V = str2num(char( poly_data(i) ));
        if vIndexed == 1
            VERTS = [VERTS; mesh_vertex(V(2:4),[])];  % Face data is currently empty here
        else 
            VERTS = [VERTS; mesh_vertex(V(1:3),[])];  % Face data is currently empty here
        end
    end
    
    % Populate faces
    zero_indexed = 0;
    for i=num_verts+3:num_verts+2+num_faces
        F = str2num(char( poly_data(i) ));
        FACES = [FACES; mesh_face(F)];
        % Auto-detect if the input file used zero-indexing 
        if min(F) < 1, zero_indexed = 1; end
    end
    
    % Of course, MATLAB does not use zero indexing, so we may have to increment
    if zero_indexed == 1
        for i=1:num_faces 
           FACES(i).verts = FACES(i).verts + 1; 
        end
    end
    
    % For each face, populate each vertex's 'faces' vector 
    for i=1:length(FACES)
       for j=1:length(FACES(i).verts)
           v = FACES(i).verts(j);
           VERTS(v).faces = [VERTS(v).faces; i];
       end
    end
    % The above should also be done for a list of edges
    % ---> it's done in bodyMesh()
    
    mesh = bodyMesh(VERTS,FACES); 

end














