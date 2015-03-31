
% An experimental scene for collision detection 

b1 = mesh_cube();
    b1.color = [.6 .5 .5];

b2 = mesh_tetrahedron();
    b2.color = [.2 .2 .8];
    b2.u = [0; 1.1; .8]; 
    %b2.quat = qt([1;0;0],0.2);
    b2.quat = qt([0;0;1],pi/6+.05);
    b2.quat = qtmultiply(qt([1;0;0],.1),b2.quat);

% Draw bodies
b1 = body_updateMesh(b1);
b1 = body_draw_init(b1);
b2 = body_updateMesh(b2);
b2 = body_draw_init(b2);
axis equal; grid on; rotate3d; view(3); hold on; 
xlabel('X'); ylabel('Y'); zlabel('Z'); 


% Draw valid edge-edge contacts
EE_drawValid(b1, b2); 


