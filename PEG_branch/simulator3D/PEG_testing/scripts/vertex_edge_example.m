

function sim = vertex_edge_example( )

    function sim = userFunction(sim)
% 
%         VolumeOverlap = 0;
%         for b1_iter=1:sim.num_bodies-1
%             for b2_iter = b1_iter+1:sim.num_bodies
%               if b2_iter < 6, continue; end
%               if cd_intersect_AABBs(sim.bodies(b1_iter),sim.bodies(b2_iter))
%                 VolumeOverlap = VolumeOverlap + volume_of_overlap(sim.bodies(b1_iter),sim.bodies(b2_iter));
%               end
%             end
%         end
%         sim.userData.VolumeOverlap(sim.step) = VolumeOverlap; 
%         sim.userData.Ccount(sim.step) = length(sim.contacts);
%         sim.userData.Icount(sim.step) = length(sim.Iconstraints); 
        
        
%         if sim.step == 1
%             movie_index = 1;
%             for r=-390:5:-30
%                view(r,30);
%                drawnow; 
%                sim.userData.startMovie(movie_index) = getframe;
%                movie_index = movie_index + 1; 
%             end
%             for dex=1:12
%                view(-30+dex*2.5,30-dex*2.5) 
%                drawnow; 
%                sim.userData.startMovie(movie_index) = getframe;
%                movie_index = movie_index + 1; 
%             end
%         end
%         
%         
%         sim.MOV(sim.step) = getframe; 
        
    end

    % Creat a box to drop objects into
    w = 3.5;
    l = 2;
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
    for i=2:length(box_bodies)
       box_bodies(i).facealpha = 0.2;  
       %box_bodies(i).visible = false; 
    end

    % Two additional bodies
    A = mesh_tetrahedron(); 
        A = scale_mesh(A,1);
        A.u = [1;0;.3];
        A.nu(1) = -1; 
        A.color = [1 0 0];
        A.quat = qt([0;0;1],pi/3);
    B = mesh_triprism();
        B = scale_mesh(B,1);
        B.u = [-1;0;.6];
        B.quat = qt([1;0;0],pi/2);


    %% Perform contact_identification / geometric tests
    sim = Simulator( 0.01 );
    sim = sim_addBody( sim, [box_bodies; A; B] ); 
    sim.draw = true;
    sim.drawContacts = true; 
    sim.MAX_STEP = 250;
    sim.userFunction = @userFunction; 


    % Dynamics selection
    dynamics = 1;
    if dynamics == 1
        sim.H_collision_detection = @get_constraints_by_configuration; 
        sim.H_dynamics = @PEG3d;
    elseif dynamics == 2
        sim.H_collision_detection = @penalty_cd; 
        sim.H_dynamics = @penalty_dynamics;
    end
    sim = sim_run( sim );



end

