
function sim = cards( )

    sim = Simulator( 0.005 );
    sim.drawContacts = true; 
    %sim.MAX_STEP = 1; 

    % Ground plane
    ground = Body_plane([0; 0; 0], [0; 0; 1]); 
        ground.mu = 0.2;
        ground.color = [.6 .6 .2];
    
    % Cards
    width = 0.5;
    length = 0.8;
    height = 0.01; 
    angle = 0.2;
    
    card = mesh_rectangularBlock(width, length, height);
    
    c1 = card; 
        c1.quat = qt([1;0.01;0], pi/2 + angle);
        c1.u = [0.01, height/2+length*sin(angle)/2, length/2*cos(angle)];
        
    c2 = card;
        c2.quat = qt( [1;0;0], -(pi/2 + angle) );
        c2.u = [0, -(height/2+length*sin(angle)/2), length/2*cos(angle)];
        c2.color = [1 0 0];


    bodies = [ground c1 c2];
    sim = sim_addBody(sim, bodies);
    
    sim = sim_run( sim ); 
end

