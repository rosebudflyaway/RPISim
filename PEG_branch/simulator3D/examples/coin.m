

function sim = coin()

    P = Body_plane([0;0;0],[0;0;1]);
    C = mesh_cylinder(25,1,.5,.05);
        C.quat = qt(rand(3,1),rand);
        C.u = [0;0;1];
        
    sim = Simulator();
    sim.drawContacts = true; 
    sim = sim_addBody(sim,[P, C]);
    sim_run(sim);

end

