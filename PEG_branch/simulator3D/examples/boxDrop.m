
function sim = boxDrop( )
    
    c1 = mesh_cube();
        c1.u = [-1;0;0]; 
        c1.dynamic = false; 
        c1.color = [.7 .7 .7];
    c2 = mesh_cube();
        c2.u = [1;0;0]; 
        c2.dynamic = false; 
        c2.color = [.7 .7 .7];
    c3 = mesh_cube();
        c3.u = [0;1;0]; 
        c3.dynamic = false; 
        c3.color = [.7 .7 .7];
    c4 = mesh_cube();
        c4.u = [0;-1;0]; 
        c4.dynamic = false; 
        c4.color = [.7 .7 .7];
    dropBox = mesh_cube();
        dropBox.u = [0;0;1.5];
        dropBox = scale_mesh(dropBox,0.999);
    bodies = [c1, c2, c3, c4, dropBox];
    sim = Simulator();
    sim = sim_addBody(sim, bodies);
    sim.drawContacts = true; 
    sim = sim_run( sim ); 
        
end

