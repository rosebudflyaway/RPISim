%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
% mesh_triprism.m 
%
% Generates a triangular prism mesh object

function triprism = mesh_triprism()

    verts = [ -1/(2*sqrt(3)) -0.5 -0.5
              -1/(2*sqrt(3)) -0.5  0.5
              -1/(2*sqrt(3))  0.5 -0.5
              -1/(2*sqrt(3))  0.5  0.5
               1/sqrt(3)      0   -0.5
               1/sqrt(3)      0    0.5 ];
           
    faces = [ 1 3 5
              2 6 4
              1 2 4
              1 4 3
              1 5 6
              1 6 2
              3 4 6
              3 6 5 ];
    
    triprism = Body_mesh( verts, faces );
    
    triprism.J = eye(3)*(1/6); 
    
end


