
% Simulates a ~ [1 x 1/2 x 1/4] "box" constructed from overlapping static cylinders.  
% Generates n spheres at various heights which fall into the box.
function M = test_box(n)

% Box
  % Bottom  (mass, radius, height)
  h = 0.05;  % height of cylinders
  r = 1/4;   % radius of cylinders
  numCyls = 0;
  P = {};
  b1 = bodyCylinder(1,r,h); b1.static = 1; b1.u=[-.5;-.25;-h/2];
  b2 = bodyCylinder(1,r,h); b2.static = 1; b2.u=[0;-.25;-h/2];
  b3 = bodyCylinder(1,r,h); b3.static = 1; b3.u=[.5;-.25;-h/2];
  b4 = bodyCylinder(1,r,h); b4.static = 1; b4.u=[.5;.25;-h/2];
  b5 = bodyCylinder(1,r,h); b5.static = 1; b5.u=[0;.25;-h/2];
  b6 = bodyCylinder(1,r,h); b6.static = 1; b6.u=[-.5;.25;-h/2];
  b7 = bodyCylinder(1,1/4,h); b7.static = 1; b7.u=[-.25;0;-h/2];
  b8 = bodyCylinder(1,1/4,h); b8.static = 1; b8.u=[.25;0;-h/2];
    P = {b1;b2;b3;b4;b5;b6;b7;b8};
    numCyls = 8;
  
  % Sides
  b9 = bodyCylinder(1,.3,h); b9.static = 1; b9.u=[-(.5+h/2);0;.124]; b9.quat = quat([0;1;0],pi/2);
  b10 = bodyCylinder(1,.3,h); b10.static = 1; b10.u=[(.5+h/2);0;.124]; b10.quat = quat([0;1;0],pi/2);
  b11 = bodyCylinder(1,.3,h); b11.static = 1; b11.u=[-.25;-(.25+h/2);.124]; b11.quat = quatmultiply(quat([0;1;0],pi/2),quat([0;0;1],pi/2));
  b12 = bodyCylinder(1,.3,h); b12.static = 1; b12.u=[.25;-(.25+h/2);.124]; b12.quat = quatmultiply(quat([0;1;0],pi/2),quat([0;0;1],pi/2));
  b13 = bodyCylinder(1,.3,h); b13.static = 1; b13.u=[-.25;.25+h/2;.124]; b13.quat = quatmultiply(quat([0;1;0],pi/2),quat([0;0;1],pi/2));
  b14 = bodyCylinder(1,.3,h); b14.static = 1; b14.u=[.25;.25+h/2;.124]; b14.quat = quatmultiply(quat([0;1;0],pi/2),quat([0;0;1],pi/2));
  P = [P; {b9;b10;b11;b12;b13;b14}];
  numCyls = numCyls + 6;
  for i=1:length(P), P{i,1}.visible = 0; end;  % Make cylinders invisible.

% Spheres 
  % Here, we look for a file containing the positions of n spheres.  If it
  % doesn't exist, then we'll create it.  We do this in order to generate a
  % random scene that can be re-used.  
  if exist( strcat('box_of_sphers_',num2str(n),'.mat') )
      disp(['Note: a file was found for ' num2str(n) ' spheres and will be used.']);
      load ( strcat('box_of_sphers_',num2str(n),'.mat') );      
      P = [P; Pspheres'];
  else
      disp(['Randomly placing ' num2str(n) ' spheres']);
      m = 0.1;           % Mass to use for spheres.
      r = .09;         % Radius to use for spheres. 
      maxHeight = 0.53;  % Maximum height to places spheres.
      for i=1:n
          % Ensure that we don't create overlapping spheres
          checkingSpheres = true;
          while checkingSpheres
              checkingSpheres = false;
              % Generate random x,y,z
              x = -(.5-r) + (1-2*r) * rand(1);     
              y = -(.25-r) + (.5-2*r) * rand(1);
              z = r + (maxHeight) * rand(1); 
              % Check position against all other spheres. 
              for s=numCyls+1 : length(P)
                  if sqrt( (P{s,1}.u(1)-x)^2 + (P{s,1}.u(2)-y)^2 + (P{s,1}.u(3)-z)^2 ) -2*r < 0
                     checkingSpheres = true; 
                  end
              end
          end
          s = bodySphere(m,r);  
          s.u = [x;y;z]; 
          %s.mu = 0; %%%%%%%%%%%%% FRICTIONLESS SPHERES %%%%%%%%%%%%%
          P = [P; {s}];  % Add sphere to the list of objects.
      end
      Pspheres = {P{numCyls+1:end,1}};
      %save(strcat('box_of_sphers_',num2str(n),'.mat'),'Pspheres');
  end

  % % Plot corners of box for reference, and box wire-frame
%     plot3( [-.5 .5 .5 -.5 -.5 .5 .5 -.5], ...
%            [-.25 -.25 .25 .25 -.25 -.25 .25 .25], ...
%            [0 0 0 0 .25 .25 .25 .25],'ro');
%     hold all;
%     plot3([-.5 .5 .5 -.5 -.5], [-.25 -.25 .25 .25 -.25], [0 0 0 0 0]); % Box bottom
%     plot3([-.5 .5 .5 -.5 -.5], [-.25 -.25 .25 .25 -.25], [.25 .25 .25 .25 .25]); % Box top 
%     plot3([-.5 -.5],[-.25 -.25],[0 .25]);  % Four corners
%     plot3([ .5  .5],[-.25 -.25],[0 .25]);
%     plot3([ .5  .5],[ .25  .25],[0 .25]);
%     plot3([-.5 -.5],[ .25  .25],[0 .25]);

% Simulate
sim = Simulation(P, .01);
sim.MAX_ITERS = 200;
sim.gravityON; 
sim.RECORD_DATA = 0;
sim.run;
end

