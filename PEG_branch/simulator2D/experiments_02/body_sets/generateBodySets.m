

function generateBodySets( )
    
    function randoBods = getBodies()
        num_bods = 350;
        N = 10; 
        randoBods(num_bods) = body_square;
        source_location = [0.5; 0.8]; 
        TriScale = 0.25; 
        for b=1:num_bods
            xa = rand(N,1);
            xa = xa - mean(xa);
            ya = rand(N,1);
            ya = ya - mean(ya); 
            ka = convhull(xa,ya);
            ka(end) = [];
            newBod = Body(xa(ka), ya(ka));  
            newBod.color = rand(1,3);
            newBod.rot = 2*pi*rand; 

            if mod(b,2) == 0
                newBod.pos = source_location;
                newBod.nu(1) = -(0.7); 
            else
                newBod.pos = [-.5; .8];
                newBod.nu(1) = (0.7); 
            end

            newBod.verts_local = TriScale * newBod.verts_local; 
            newBod.J = 0.003; 
            newBod.mass = TriScale;  

            newBod.Fext = [0;-9.8*newBod.mass;0];
            randoBods(b) = newBod;  
        end
    end

    Bods = getBodies();
    
%     B = Bods(1:25);
%     save('experiments_02/body_sets/random_bodies_a.mat','B');
%     B = Bods(26:50);
%     save('experiments_02/body_sets/random_bodies_b.mat','B');
%     B = Bods(51:75);
%     save('experiments_02/body_sets/random_bodies_c.mat','B');
%     B = Bods(76:100);
%     save('experiments_02/body_sets/random_bodies_d.mat','B');
%     B = Bods(101:125);
%     save('experiments_02/body_sets/random_bodies_e.mat','B');
%     B = Bods(126:150);
%     save('experiments_02/body_sets/random_bodies_f.mat','B');
        B = Bods(251:275);
        save('experiments_02/body_sets/random_bodies_g.mat','B');
%     B = Bods(176:200);
%     save('experiments_02/body_sets/random_bodies_h.mat','B');
        B = Bods(276:300);
        save('experiments_02/body_sets/random_bodies_i.mat','B');
%     B = Bods(226:250);
%     save('experiments_02/body_sets/random_bodies_j.mat','B');

end












