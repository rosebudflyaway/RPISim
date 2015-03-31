

B1 = obj_mesh(mesh_vertex([0;0;0],[]),[]);
    B1.u = [0.1;0.1;1];
    B1.update_world_position;
    B1.Fext = [0;0;-9.8]*B1.mass; 
%B2 = mesh_tetrahedron;
B2 = mesh_read_poly_file('square.poly');
    %B2.u = [0; 0; 1]; 
    %B2.quat = quat([1;0;0],pi/2);
    B2.update_world_position; 
    B2.static = 1;
    


B2.draw; axis equal; hold on; xlabel('X'); ylabel('Y'); zlabel('Z'); 
plot3(B1.u(1),B1.u(2),B1.u(3),'ro'); grid on;

[C_i num_subContacts_i] = cda_collide_poly_poly(B1, 1, B2, 2, 0)

C_i
C_i.psi_n

