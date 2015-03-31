
% Test mesh objects
% jw

%clear all; close all;
clf;

C = mesh_cube();
T = mesh_tetrahedron();
    T.u = [1;2;2];
I = mesh_icosahedron();
    I.u = [-3;0;1];
H = mesh_octahedron();
    H.u = [2;-2;0];
D = mesh_dodecahedron();
    D.u = [0;-3;-2];

for i=1:100
   % Rotate Cube 
   C.quat = quat([0;0;1],i/10); 
   
   % Move tetrahedron up and down
   T.u(3) = 2+sin(i/10); 
   
   % Rotate the icosahedron
   I.quat = quat(rand(1,3),i/10);
   
   % Move around in y
   H.u(2) = -2 + sin(i/5);
   H.quat = quat([1;0;0],i/20);
   
   % Meh
   D.u(2:3) = [-3;-2] + [cos(i/5);sin(i/5)];
   
   % Draw the shapes
   C.draw(); hold on;
   T.draw(); 
   I.draw();
   H.draw();
   D.draw();
   
   axis([-4 4 -4 4 -3 5]); 
   axis square;
   view(3)
   %drawnow
   pause(.1);
end
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 

