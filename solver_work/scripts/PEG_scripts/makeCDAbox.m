%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
% makeCDAbox.m 
%

% Generates a "box" and sets up some number of tetrahedron to fall into it.
function makeCDAbox( numObjects, fric_dirs )

    % Scale of scene
    SCL = 10;

    % Label verts from view looking down on box:
    %     d*---------*c
    %      |         |
    %      |         |
    %     a*---------*b    Center around x=0, y=0

    % Defines vert locations
    dx = SCL * 0.5;
    dy = SCL * 0.25;
    dz = SCL * 0.1;
    
    % Bottom of box
    box1 = mesh_read_poly_file('square.poly');
        box1.setStatic(true);
        box1.verts(1).local_coords = [-dx;-dy;0];
        box1.verts(2).local_coords = [ dx;-dy;0];
        box1.verts(3).local_coords = [ dx; dy;0];
        box1.verts(4).local_coords = [-dx; dy;0];
    
    % Shorter ends
    box2 = mesh_read_poly_file('square.poly');
        box2.u = [-dx; 0; dz];
        box2.quat = quat([0;1;0],-pi/2); 
        box2.setStatic(true);
        box2.verts(1).local_coords = [-dz;-dy;0];
        box2.verts(2).local_coords = [ dz;-dy;0];
        box2.verts(3).local_coords = [ dz; dy;0];
        box2.verts(4).local_coords = [-dz; dy;0];
    box3 = mesh_read_poly_file('square.poly');
        box3.u = [ dx; 0; dz];
        box3.quat = quat([0;1;0],pi/2);
        box3.setStatic(true);
        box3.verts(1).local_coords = [-dz;-dy;0];
        box3.verts(2).local_coords = [ dz;-dy;0];
        box3.verts(3).local_coords = [ dz; dy;0];
        box3.verts(4).local_coords = [-dz; dy;0];
    
    % Larger sides
    box4 = mesh_read_poly_file('square.poly');
        box4.u = [0;  dy; dz];
        box4.quat = quat([1;0;0],-pi/2);
        box4.setStatic(true);
        box4.verts(1).local_coords = [-dx;-dz;0];
        box4.verts(2).local_coords = [ dx;-dz;0];
        box4.verts(3).local_coords = [ dx; dz;0];
        box4.verts(4).local_coords = [-dx; dz;0];
    box5 = mesh_read_poly_file('square.poly');
        box5.u = [0; -dy; dz];
        box5.quat = quat([1;0;0], pi/2);
        box5.setStatic(true);
        box5.verts(1).local_coords = [-dx;-dz;0];
        box5.verts(2).local_coords = [ dx;-dz;0];
        box5.verts(3).local_coords = [ dx; dz;0];
        box5.verts(4).local_coords = [-dx; dz;0];
    
    Bodies = {  box1; box2; box3; box4; box5 };
    for b=1:5, Bodies{b}.faceAlpha = 0.2; end 
         
    % Generate tetrahedra
    n = numObjects; 
    disp(['Randomly placing ' num2str(n) ' tetrahdra and spheres...']);
      r = .09;          % Scale to use for objects. 
      m = r*SCL;           % Mass to use for objects.
      maxHeight = SCL*1.1;  % Maximum height to places objects.
      dx = dx-SCL*r;   
      dy = dy-SCL*r;   
      for i=1:n
          % Ensure that we don't create overlapping objects
          checkingObjects = true;
          while checkingObjects
              checkingObjects = false;
              % Generate random x,y,z
              x = -dx + 2*dx*rand(1);   
              y = -dy + 2*dy*rand(1); 
              z = 1 + (maxHeight-1) * rand(1); 
              
              % Check position against all other objects. 
              for s=5+1 : length(Bodies)
                  if sqrt( (Bodies{s,1}.u(1)-x)^2 + (Bodies{s,1}.u(2)-y)^2 + (Bodies{s,1}.u(3)-z)^2 ) -SCL*r < 0
                     checkingObjects = true; 
                  end
              end
          end
          
          % Randomize tetrahedra and spheres
          if i > .7*n
             body = bodySphere(m,r*SCL/2.2);
          else
             body = mesh_tetrahedron();            % Create object
             body.scale(r*SCL);                    % Scale
          end;
          body.u = [x;y;z];                        % Position
          body.quat = quat(rand(1,3),2*pi*rand);   % Rotation
          Bodies = [Bodies; {body}];               % Add object to the list of objects
      end
   
    SIM = Simulation(Bodies, 0.01);
    SIM.formulation('PEG'); 
    SIM.num_fricdirs = fric_dirs;
    SIM.gravityON(); 
%         SIM.setRecord(true);
%         SIM.disableGUI(); 
%         SIM.MAX_ITERS = 200; 
    SIM.run;
    
end

