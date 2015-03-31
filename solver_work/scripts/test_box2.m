%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
% test_box2.m 
%
% Simulates a ~ [1 x 1/2 x 1/4] "box" constructed from overlapping static cylinders.  
% Generates n spheres at various heights which fall into the box.
function M = test_box2(n)

% Box
  % Bottom  (mass, radius, height)
  h = 0.05;  % height of cylinders
  r = 1/4;   % radius of cylinders
  numCyls = 0;
  P = {};
  b0 = bodyCylinder(1,.6,h); b0.static = 1; b0.u=[0;0;-h/2];
    P = {b0};
    numCyls = 1;
  
  % Sides
  b9 = bodyCylinder(1,.3,h); b9.static = 1; b9.u=[-(.5+h/2);0;.124]; b9.quat = quat([0;1;0],pi/2);
  b10 = bodyCylinder(1,.3,h); b10.static = 1; b10.u=[(.5+h/2);0;.124]; b10.quat = quat([0;1;0],pi/2);
  b11 = bodyCylinder(1,.6,h); b11.static = 1; b11.u=[0;(.25+h/2);.124]; b11.quat = quatmultiply(quat([0;1;0],pi/2),quat([0;0;1],pi/2));
  b12 = bodyCylinder(1,.6,h); b12.static = 1; b12.u=[0;-(.25+h/2);.124]; b12.quat = quatmultiply(quat([0;1;0],pi/2),quat([0;0;1],pi/2));
    P = [P; {b9;b10;b11;b12}];
    numCyls = numCyls + 4;
  for i=1:length(P), P{i,1}.visible = 0; end;  % Make cylinders invisible.
  
  b13 = bodyCylinder(1,.21,.07); b13.static = 1; b13.u=[.2;0;.5]; b13.quat=quat([0;1;0],pi/5);
  b14 = bodyCylinder(1,.21,.07); b14.static = 1; b14.u=[-.2;0;.7]; b14.quat=quat([0;1;0],-pi/5);
  b15 = bodyCylinder(1,.25,.07); b15.static = 1; b15.u=[.2;0;.9]; b15.quat=quat([0;1;0],pi/5);
  P = [P; {b13;b14;b15}]; 
    numCyls = numCyls + 3;
  

% Spheres 
  % Here, we look for a file containing the positions of n spheres.  If it
  % doesn't exist, then we'll create it.  We do this in order to generate a
  % random scene that can be re-used.  
  if exist( strcat('box_of_spheres_',num2str(n),'.mat') )
      disp(['Note: a file was found for ' num2str(n) ' spheres and will be used.']);
      load ( strcat('box_of_spheres_',num2str(n),'.mat') );      
      P = [P; Pspheres'];
  else
      disp(['Randomly placing ' num2str(n) ' spheres']);
      m = .2;           % Mass to use for spheres.
      r = .025;         % Radius to use for spheres. 
      maxHeight = 1.2;  % Maximum height to places spheres.
      for i=1:n
          % Ensure that we don't create overlapping spheres
          checkingSpheres = true;
          while checkingSpheres
              checkingSpheres = false;
              % Generate random x,y,z
              x = .36 * rand(1);     
              y = -.18 + (.36) * rand(1);
              z = 1 + (maxHeight-1) * rand(1); 
              
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
      save(strcat('box_of_sphers_',num2str(n),'.mat'),'Pspheres');
  end

%% Simulate
sim = Simulation(P,.01);  
 %sim.formulation('LCP');
 %sim.solver('Lemke');

sim.run();


end


