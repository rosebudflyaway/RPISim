
%% Define bodies A and B
% N = 15;
% xa = rand(N,1);
% ya = rand(N,1);
% ka = convhull(xa,ya);
% ka(end) = [];
% A = Body(xa(ka), ya(ka));  
% A.color = [1 0 0];
% A.pos(1) = rand-0.5;
% A.rot = 2*pi*rand; 
% 
% xb = rand(N,1);
% yb = rand(N,1);
% kb = convhull(xb,yb);
% kb(end) = []; 
% B = Body(xb(kb), yb(kb));  
% B.pos = [0 1];
% B.rot = 2*pi*rand;
% 
% 
% sim = Simulator();
% sim = sim_addBody(sim,[A B]);
% sim_draw_init(sim); 



    T1 = body_triangle; 
        T1.color = [1 0 0];         
        %T1.rot = .05; 
    
    T2 = body_triangle;  
        %T2.pos = [0.05;3];
        %T2.pos = [3;.1]; 
        T2.pos = [2.6;0]; 
        %T2.rot = pi; 
        %T2.rot = pi/6;
        T2.nu(1) = -1.0; 
        
        
        %T1.rot = .2;  T2.rot = -0.2;  T1.pos = [-.5 0];
        
    sim = Simulator( .25 );
    sim = sim_addBody( sim, [T1 T2] );
    
    sim.gravity = false; 
    sim_draw_init(sim); 


A = sim.bodies(1);
B = sim.bodies(2); 
%% Highlight vertices
for va=1:A.num_verts
   plot(A.verts_world(va,1),A.verts_world(va,2),'o'); 
end
for vb=1:B.num_verts
   plot(B.verts_world(vb,1),B.verts_world(vb,2),'o'); 
end

if intersecting_poly_poly(A.verts_world, B.verts_world)
   title('Intersection!'); 
end

%%

%sim = PEG_collision_detection( sim );
sim = get_all_contacts_2d( sim); 
disp(['Found ' num2str(length(sim.contacts)) ' total contacts']);
sim = PEG_constraints_from_contacts( sim, -0.25 );  % eps_theta = 0.25
disp([' Kept ' num2str(length(sim.contacts)) ' total contacts']);
sim = sim_drawContacts( sim ); 

newNU = PEG_dynamics( sim ); 






