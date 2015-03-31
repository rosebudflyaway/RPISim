




function sim = bunnyTest()

    function sim = userFunction(sim)
        if ~mod(sim.step+1,25) 
            newBod = mesh_icosahedron();
            newBod = scale_mesh(newBod,0.2);
            newBod.u = [-1.25+2.5*rand;-.75+1.5*rand;4];
            newBod.Fext(3) = -9.81 * newBod.mass; 
            newBod.color = rand(1,3);
            newBod.quat = qt([1 2 3],.2);
            if sim.draw
                newBod = body_draw_init(newBod);
            end
            sim = sim_addBody( sim, newBod );
        end
        
         sim.MOV(sim.step) = getframe; 
        
    end


bunny = mesh_read_poly_file('bunny_1k.obj'); 
    bunny = scale_mesh(bunny,10); 
    bunny.quat = qt([1;0;0],pi/2); 
    bunny.color = [.5 .5 .5];
    bunny.dynamic = false; 
    

%% Perform contact_identification / geometric tests
sim = Simulator(0.01);
sim = sim_addBody( sim, bunny ); 
sim.draw = true;
sim.drawContacts = true; 
sim.MAX_STEP = 1000;
sim.userFunction = @userFunction; 


% Dynamics selection
sim.H_collision_detection = @get_constraints_by_configuration; 
sim.H_dynamics = @PEG3d;
sim = sim_run( sim );

save('bunnyMOV2.mat','sim'); 

%V = volume_of_overlap(sim.bodies(1),sim.bodies(2))

end







