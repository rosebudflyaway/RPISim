

function sim = ST_trap_vertex_edge( )

    A = mesh_cube();
        A.color = [0.9 .3 .3];
        A.u = [1;0;1];
        A.facealpha = 1;
    B = mesh_rectangularBlock(3,3,1);
        B.facealpha = 1;
        B.color = [.2 .2 1];
    
    sim = Simulator(.01);
    sim = sim_addBody(sim,[A B]);
    
    sim.MAX_STEP = 0;
    sim = sim_run(sim); 
    

end

