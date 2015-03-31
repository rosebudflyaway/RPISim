

function sim = testGeometry_5(dynamics, timestep)

    function sim = userFunction(sim)
        %if ~mod(sim.step+1,50)    
        if mod( sim.step-1, round(.5/sim.h) ) == 0
            newBod = mesh_icosahedron();
            newBod = scale_mesh(newBod,0.5);
            newBod.u = [3;0;2];
            newBod.nu(1) = -5;
            newBod.Fext(3) = -9.81 * newBod.mass; 
            newBod.color = rand(1,3);
            newBod.quat = qt([1 2 3],.2);
            if sim.draw
                newBod = body_draw_init(newBod);
            end
            sim = sim_addBody( sim, newBod );
        end
         
        VolumeOverlap = 0;
        for b1_iter=1:sim.num_bodies-1
            for b2_iter = b1_iter+1:sim.num_bodies
              if b2_iter < 6, continue; end
              if cd_intersect_AABBs(sim.bodies(b1_iter),sim.bodies(b2_iter))
                [V_temp,P] = volume_of_overlap(sim.bodies(b1_iter),sim.bodies(b2_iter));
                VolumeOverlap = VolumeOverlap + V_temp;
              end
            end
        end
        sim.userData.VolumeOverlap(sim.step) = VolumeOverlap; 
        sim.userData.Ccount(sim.step) = length(sim.contacts);
        sim.userData.Icount(sim.step) = length(sim.Iconstraints); 
        
%         sim.MOV(sim.step) = getframe; 
        
    end

% Creat a box to drop objects into
w = 10;
l = 4;
h = .25;
box_1 = mesh_rectangularBlock(w+h,l+h,h);
    box_1.u = [0;0;-h/2];
    box_1.dynamic = false; 
    box_1.color = [.4 .4 .4];
box_2 = mesh_rectangularBlock(h,l+h,l);
    box_2.u = [-w/2;0;l/2-h]; 
    box_2.dynamic = false; 
    box_2.color = [.4 .4 .4];
box_3 = mesh_rectangularBlock(h,l+h,l);
    box_3.u = [w/2;0;l/2-h];
    box_3.dynamic = false; 
    box_3.color = [.4 .4 .4];
box_4 = mesh_rectangularBlock(w+h,h,l);
    box_4.u = [0;-l/2;l/2-h];
    box_4.dynamic = false; 
    box_4.color = [.4 .4 .4];
box_5 = mesh_rectangularBlock(w+h,h,l);
    box_5.u = [0;l/2;l/2-h];
    box_5.dynamic = false; 
    box_5.color = [.4 .4 .4];
    
box_bodies = [box_1; box_2; box_3; box_4; box_5]; 
for i=1:length(box_bodies)
   box_bodies(i).facealpha = 0.05;  
   %box_bodies(i).visible = false; 
end



%% Perform contact_identification / geometric tests
sim = Simulator(timestep);
sim = sim_addBody( sim, [box_bodies] ); 
sim.draw = true;
sim.drawContacts = true; 
%sim.MAX_STEP = 8/sim.h;
sim.MAX_STEP = 5/sim.h;
sim.userFunction = @userFunction; 


% Dynamics selection
if strcmp(dynamics,'PEG')
    sim.H_collision_detection = @get_constraints_by_configuration; 
    sim.H_dynamics = @PEG3d;
elseif strcmp(dynamics,'Penalty')
    sim.H_collision_detection = @penalty_cd; 
    sim.H_dynamics = @mLCPdynamics;
else
    error('Dynamic type?');
end
sim.FRICTION = false; 
sim = sim_run( sim );

end























