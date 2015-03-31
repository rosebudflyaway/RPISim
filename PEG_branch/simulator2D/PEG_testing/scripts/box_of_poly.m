

function sim = box_of_poly( dynamicsType, h, bodyDATA)

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    function sim = polySource( sim )
        
        if sim.step <= 1
           % Generate random bodies
           %if exist(bodyFile,'file')
               %load(bodyFile); 
               sim.userData.randoBods = bodyDATA; 
%            else
%                randoBods(100) = body_square;
%                source_location = [0.5; 0.8]; 
%                TriScale = 0.25; 
%                N = 10;
%                for b=1:100
%                     xa = rand(N,1);
%                     xa = xa - mean(xa);
%                     ya = rand(N,1);
%                     ya = ya - mean(ya); 
%                     ka = convhull(xa,ya);
%                     ka(end) = [];
%                     newBod = Body(xa(ka), ya(ka));  
%                     newBod.color = rand(1,3);
%                     newBod.rot = 2*pi*rand; 
% 
%                     if mod(b,2) == 0
%                         newBod.pos = source_location;
%                         newBod.nu(1) = -(0.7); 
%                     else
%                         newBod.pos = [-.5; .8];
%                         newBod.nu(1) = (0.7); 
%                     end
% 
%                     newBod.verts_local = TriScale * newBod.verts_local; 
%                     newBod.J = 0.005; 
%                     newBod.mass = TriScale;  
% 
%                     newBod.Fext = [0;-9.8*newBod.mass;0];
%                     randoBods(b) = newBod;  
%                end
%                save('randomBodies.mat','randoBods');
%                sim.userData.randoBods = randoBods; 
%            else
%                error('Bad body file');
%            end
           sim.userData.bod_index = 1;
        end
        % randoBods already exists
        if mod( sim.step-1, round(.25/sim.h) ) == 0
            newBod = sim.userData.randoBods(sim.userData.bod_index);
            sim.userData.bod_index = sim.userData.bod_index + 1;
            if sim.draw
                newBod = body_draw_init(newBod);
            end
            
            
            % Bounding box
            if sim.drawBoundingBoxes
                Vbb = zeros(8,2); 
                for b=1:sim.num_bodies
                    M = sim.bodies(b).AABB_max;
                    m = sim.bodies(b).AABB_min; 
                    Vbb(1,:) = [m(1) m(2)];  % Verts of BBox
                    Vbb(2,:) = [m(1) M(2)];
                    Vbb(3,:) = [M(1) M(2)];
                    Vbb(4,:) = [M(1) m(2)];
                    Vbb(5,:) = [m(1) m(2)];
                    Vbb(6,:) = [m(1) M(2)];
                    Vbb(7,:) = [M(1) M(2)];
                    Vbb(8,:) = [M(1) m(2)];
                    Ebb = [ Vbb(1,:); Vbb(2,:)
                            Vbb(3,:); Vbb(4,:)
                            Vbb(1,:); Vbb(5,:)
                            Vbb(6,:); Vbb(7,:)
                            Vbb(8,:); Vbb(5,:)
                            Vbb(6,:); Vbb(2,:)
                            Vbb(3,:); Vbb(7,:)
                            Vbb(8,:); Vbb(4,:) ];
                    newBod.bboxHandle = plot(Ebb(:,1),Ebb(:,2),'b');
                end 
            end
            
            sim = sim_addBody( sim, newBod );
        end
        
        
        %% Polygon overlap
        this_error = 0;
        
        for b1id = 1 : length(sim.bodies)-1
           for b2id = b1id+1 : length(sim.bodies)
               if b1id <= 3 && b2id <= 3 || ~cd_intersect_AABBs(sim.bodies(b1id),sim.bodies(b2id)), continue; end
               
               overlap_poly = sutherlandHodgman( sim.bodies(b1id).verts_world, ...
                                                 sim.bodies(b2id).verts_world );
               if ~isempty(overlap_poly)
                  this_error = this_error + polyarea(overlap_poly(:,1), overlap_poly(:,2)); 
               end
           end
        end
        
        if sim.step < 2
            sim.userData.polyError = [];
        else
            sim.userData.polyError = [sim.userData.polyError this_error];
        end
        
        
        %% Last step
        if sim.step == sim.MAX_STEP
           figure; 
           plot(sim.userData.polyError,'linewidth',2)
           title('Area of overlap during simulation');
           xlabel('Time step (s)');
           ylabel('Total area of overlap');
        end
        
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    
    dw = .2;
    box_color = [.5 .5 .5]; 
    box_bottom = Body([1 1 -1 -1]', [-dw dw dw -dw]'); 
        box_bottom.dynamic = false; 
        box_bottom.pos = [0; -dw]; 
        box_bottom.color = box_color;
        box_bottom.faceAlpha = 1; 
        
    box_left = Body([dw dw -dw -dw]', [-1 1 1 -1]'); 
        box_left.dynamic = false; 
        box_left.pos = [-1; 1-2*dw];
        box_left.color = box_color;
        box_left.faceAlpha = 1; 
        
    box_right = Body([dw dw -dw -dw]', [-1 1 1 -1]');
        box_right.dynamic = false; 
        box_right.pos = [1; 1-2*dw];
        box_right.color = box_color;
        box_right.faceAlpha = 1; 
        
    sim = Simulator( h );
    sim.userFunction = @polySource; 
    sim = sim_addBody( sim, [box_bottom box_left box_right] ); 
    sim.draw = true; 
    sim.drawContacts = true; 
    sim.drawBoundingBoxes = false; 
    sim.FRICTION = false; 
                                    sim.MAX_STEP = 405; 
    %sim.MAX_STEP = 1000;
    %sim.MAX_STEP = 5/sim.h;
    sim.userData.STcount = zeros(1,sim.MAX_STEP);
    
    dynamics = dynamicsType;
    switch dynamics
        case 1  %% Stewart-Trinkle
            sim.H_collision_detection = @collision_detection; 
            sim.H_dynamics = @mLCPdynamics; 
        case 2  %% PEG
            sim.H_collision_detection = @peg_collision_detection; 
            sim.H_dynamics = @PEGdynamics; 
        case 3  %% mPEG
            sim.H_collision_detection = @mPEG_collision_detection; 
            sim.H_dynamics = @mPEGdynamics; 
        case 4 %% NEW PEG
            sim.H_collision_detection = @get_all_contacts_2d;
            sim.H_dynamics = @PEG_dynamics; 
        case 5 %% Heuristic PEG
            sim.H_collision_detection = @get_all_contacts_2d;
            sim.H_dynamics = @PEG2_dynamics; 
        case 6 %% Penalty method
            sim.H_collision_detection = @penaltyCD;
            sim.H_dynamics = @mLCPdynamics; 
    end
    sim = sim_run( sim ); 
    
    
    
    
end




























