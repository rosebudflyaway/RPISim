
% Height 
%height = 18;
height = 4;

scale = 100;
space_eps = 0.0005; % Half mm between blocks
space_eps = scale*space_eps;  
dx = space_eps + 0.075*scale;   % Dimensions of a Jenga block
dy = space_eps + 0.025*scale; 
dz = space_eps + 0.015*scale; 
mass = 0.05;        % TODO: find correct mass
mu = 0.0;          % TODO: find correct friction

% Base 
base = mesh_cube();
base.scale(0.2*scale);
base.setStatic( true );
base.u(3) = -.1*scale;
base.faceAlpha = 1.0; 
bodies = {base};
for layer = 0:height-1
    if mod(layer,2) == 0
        q = qt([0;0;1],pi/2);   % Rotation about z
        % First block
        block1 = mesh_read_poly_file('Jenga.poly');
        block1.u = [-dy; 0; dz/2 + layer*dz]; 
        block1.quat = q;
        block1.scale(scale);
        % Second block
        block2 = mesh_read_poly_file('Jenga.poly');
        block2.u = [0; 0; dz/2 + layer*dz]; 
        block2.quat = q;
        block2.scale(scale);
        % Third block
        block3 = mesh_read_poly_file('Jenga.poly');
        block3.u = [dy; 0; dz/2 + layer*dz]; 
        block3.quat = q;
        block3.scale(scale);
    else
        % First block
        block1 = mesh_read_poly_file('Jenga.poly');
        block1.u = [0; -dy; dz/2 + layer*dz]; 
        block1.scale(scale);
        % Second block
        block2 = mesh_read_poly_file('Jenga.poly');
        block2.u = [0; 0; dz/2 + layer*dz]; 
        block2.scale(scale);
        
            if layer == 1
               block2.Aext(1) = 10; 
               %block2.scale(.9);
               %block2.u(3) = 8;
            end
        
        % Third block
        block3 = mesh_read_poly_file('Jenga.poly');
        block3.u = [0; dy; dz/2 + layer*dz]; 
        block3.scale(scale);
    end
    
    bodies = [bodies; {block1; block2; block3}];
end

Sim = Simulation( bodies, 0.1 );
Sim.formulation('PEG');
Sim.gravityON(); 
Sim.MAX_ITERS = 50;
Sim.disableGUI();
Sim.setRecord( true ); 
Sim.run(); 



