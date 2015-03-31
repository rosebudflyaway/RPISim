%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
% does_EdgeCrossEdge.m 
%
% Used during edge-edge collision detection.  If two edges do not cross,
% then we don't want to include that contact.  For example, a slightly
% smaller cube sitting centered on a larger cube.  If the edges are within
% epsilon, we still wish to ignore them since the vertex-face interaction
% will cover the penetration constraint.  Further, we wish to limit the
% number of contacts we have.  Lastly, we run into problems with edge-edge
% gap calculations when the edges are not crossing!  

% Returns true if the edges cross, false otherwise.  Edges are not
% considered crossed if their vertices can be projected onto a single face
% of the other body.  

% Inputs:
%   B1  - First body
%   B2  - Second body
%   e1x - Edge INDEX! of edge on B1
%   e2x - Edge INDEX of edge on B2

% TODO: Because of the (necessary) non-zero epsilon, this currently
% TODO: This function is WAY too sensitive on the value of 'ep' below.  
% returns a false positive for edges with coplanar faces (e.g. checkered floor).  
function doesCross = does_EdgeCrossEdge( B1,B2,e1x,e2x )
    doesCross = true;
    ep = -.01;       % A value of epsilon 
    %ep = 0;  % I'm wary of this value being zero...
    
    e1 = B1.edges(e1x); 
    e2 = B2.edges(e2x); 
    
    %% Project e1 verts onto B2 faces
    p1 = B1.verts(e1.verts(1)).world_coords;    % e1 end-points
    p2 = B1.verts(e1.verts(2)).world_coords;
    
    f1norm = B2.faces(e2.faces(1)).normal;      % B2 face normals
    f2norm = B2.faces(e2.faces(2)).normal;
    b2point = B2.verts(e2.verts(1)).world_coords;
    
    v1 = p1-b2point;
    v2 = p2-b2point; 
    
    % If dot product with a face normal is < 0 for both points, it doesn't
    % cross the edge with that face.
    
    % Face 1
    f1_dot1 = dot3(f1norm, v1);
    f1_dot2 = dot3(f1norm, v2);
    
    % Face 2
    f2_dot1 = dot3(f2norm, v1);
    f2_dot2 = dot3(f2norm, v2); 
    
    if f1_dot1 <= ep && f1_dot2 <= ep || f2_dot1 <= ep && f2_dot2 <= ep
        doesCross = false; 
        return; 
    end
    
    % For debugging purposes 
    clear p1 p2 f1norm f2norm b2point v1 v2 f1_dot1 f1_dot2 f2_dot1 f2_dot2;
    
    %% Project e2 verts onto B1 faces
    p1 = B2.verts(e2.verts(1)).world_coords;
    p2 = B2.verts(e2.verts(2)).world_coords; 
    
    f1norm = B1.faces(e1.faces(1)).normal;
    f2norm = B1.faces(e1.faces(2)).normal;
    b1point = B1.verts(e1.verts(1)).world_coords; 
    
    v1 = p1-b1point;
    v2 = p2-b1point; 
    
    % Face 1
    f1_dot1 = dot3(f1norm, v1);
    f1_dot2 = dot3(f1norm, v2);
    
    % Face 2
    f2_dot1 = dot3(f2norm, v1);
    f2_dot2 = dot3(f2norm, v2); 
    
    ep = -.01;       % A value of epsilon 
    if f1_dot1 <= ep && f1_dot2 <= ep || f2_dot1 <= ep && f2_dot2 <= ep
        doesCross = false; 
        return; 
    end

end


























