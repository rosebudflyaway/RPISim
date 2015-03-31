  
 % test the penalty method, using a lot of spheres 

% Simulates a ~ [aa x bb x cc] "box" constructed from overlapping static cylinders.  
% Generates n spheres at various heights which fall into the box.
function [] = test_fixed_point(n)
% dimension of the box
% aa = 87; 
% bb = 53.2; 
% cc = 45;
% h = 28;  % height of cylinders % the thickness of the wall 

aa = 30; 
bb = 15; 
cc = 10;
h = 10;  % height of cylinders % the thickness of the wall 
 
% Box
% Bottom  (mass, radius, height) 
  rb = 0.5*sqrt(aa*aa + bb*bb) + 0.5*min(aa, bb);
  b1 = bodyCylinder(1,rb,h); b1.static = 1; b1.u=[0;0;-h/2];  

% Sides
% yz plane 
  ryz = 0.5*sqrt(bb*bb+cc*cc) + 0.5*min(bb, cc);
  b2 = bodyCylinder(1,ryz,h); b2.static = 1; b2.u=[-(aa/2+h/2);0;cc/2]; b2.quat = quat([0;1;0],pi/2);
  b3 = bodyCylinder(1,ryz,h); b3.static = 1; b3.u=[ (aa/2+h/2);0;cc/2]; b3.quat = quat([0;1;0],pi/2);
% xz plane 
  rxz = 0.5*sqrt(aa*aa+cc*cc) + 0.5*min(aa, cc);
  b4 = bodyCylinder(1,rxz,h); b4.static = 1; b4.u=[0;-(bb/2+h/2);cc/2]; b4.quat = quatmultiply(quat([0;1;0],pi/2),quat([0;0;1],pi/2));
  b5 = bodyCylinder(1,rxz,h); b5.static = 1; b5.u=[0; (bb/2+h/2);cc/2]; b5.quat = quatmultiply(quat([0;1;0],pi/2),quat([0;0;1],pi/2));
  P = {b1;b2;b3;b4;b5}; 
  
  
 for i=1:length(P), P{i,1}.visible = 0; end;  % Make cylinders invisible.
 numCyls = 5;

 % Spheres
   %density = 2.5e3;    
   %r = 0.03;         % Radius of spheres.
   %m = density*(4/3 * pi * r^3);  % Mass of spheres.
   %maxHeight = 0.25;  % Maximum height to places spheres.
   
   % the initial position
   Pspheres = [];
% Simulate    
   disp('This is the menu: ');
   fprintf('1: Lemke \t  2: PATH \t  3: mlcp_fixed_point  \t  4: mncp_fixed_point_pgs \t  5: penalty  \t  6: mingres \t 7: Fischer-Newton \t 8: pyramid fixed\n');
   ch = input('Please enter the solver you will choose : ');
   switch(ch)
       case 1
           solver_name = 'Lemke';
           Formulation = 'LCP';
           step_size = 0.01;
       case 2
           solver_name = 'PATH';
           Formulation = 'mLCP';
           step_size = 1e-2;
       case 3
           solver_name = 'mlcp_fixed_point';
           Formulation = 'mLCP';
           step_size = 1e-3;   
       case 4
           solver_name = 'mncp_fixed_point_pgs';
           Formulation = 'mNCP';
           step_size = 1e-3;
       case 5
           solver_name = 'penalty';
           step_size = 1e-3;
       case 6
           solver_name = 'mingres';
           Formulation = 'LCP';
           step_size = 1e-3;
       case 7
           solver_name = 'fischer_newton';
           Formulation = 'LCP';
           step_size = 1e-3;
       case 8
           solver_name = 'pyramid_fixed_point';
           Formulation = 'LCP';
           step_size = 1e-3;
       otherwise
           disp('Please refer to the menu for inputs'); 
           fprintf('1: Lemke\t 2: PATH \t 3: mlcp_fixed_point  \t  4: mncp_fixed_point \t  5: penalty  \t  6: mingres \t 7: Fischer-Newton \t 8: pyramid fixed\n');
   end
   disp(solver_name);
   
   
   sim = Simulation(P, step_size);
   sim.SOLVER = solver_name;
   sim.FORMULATION = Formulation;
 
%  if (sim.RECORD_DATA == 0)
%      sim.DRAW = 1;
%      figure(1); hold on; axis equal; grid on;
%      xlabel('X'); ylabel('Y'); zlabel('Z');
%      % % view(3) sets the default three-dimensional view az = -37.5, e1 = 30
%      view(3);
%      % % Plot corners of box for reference, and box wire-frame
%      plot3( [-aa/2 aa/2 aa/2 -aa/2 -aa/2 aa/2 aa/2 -aa/2], ...
%          [-bb/2 -bb/2 bb/2 bb/2 -bb/2 -bb/2 bb/2 bb/2], ...
%          [0 0 0 0 cc cc cc cc],'ro');
%      plot3([-aa/2 aa/2 aa/2 -aa/2 -aa/2], [-bb/2 -bb/2 bb/2 bb/2 -bb/2], [0 0 0 0 0], 'r'); % Box bottom
%      plot3([-aa/2 aa/2 aa/2 -aa/2 -aa/2], [-bb/2 -bb/2 bb/2 bb/2 -bb/2], [cc cc cc cc cc], 'g'); % Box top
%      plot3([-aa/2 -aa/2],[-bb/2 -bb/2],[0 cc]);  % Four edges
%      plot3([ aa/2  aa/2],[-bb/2 -bb/2],[0 cc]);
%      plot3([ aa/2  aa/2],[ bb/2  bb/2],[0 cc]);
%      plot3([-aa/2 -aa/2],[ bb/2  bb/2],[0 cc]);
%  else
%      sim.DRAW = 0;
%  end
 
tic; 
%count = 1;
% simulation start
%obj.STEP = 0;
%count_flag = true
if exist( strcat('./box_of_spheres/box_of_sphers_',num2str(n),'.mat') )
      disp(['Note: a file was found for ' num2str(n) ' spheres and will be used.']);
      load ( strcat('./box_of_spheres/box_of_sphers_',num2str(n),'.mat') );      
      P = [P; Pspheres'];
else
      disp(['Randomly placing ' num2str(n) ' spheres']);
      m = 20;           % Mass to use for spheres.
      r = 2.5;         % Radius to use for spheres. 
      maxHeight = 9;  % Maximum height to places spheres.
      for i=1:n
          % Ensure that we don't create overlapping spheres
          checkingSpheres = true;
          while checkingSpheres
              checkingSpheres = false;
              % Generate random x,y,z
             x = -(aa/2-r) + ((aa/2-r)+(aa/2-r)) * rand(1);  
             y = -(bb/2-r) + ((bb/2-r)+(bb/2-r)) * rand(1);
             z = r + (maxHeight-r) * rand(1);  
              
              % Check position against all other spheres. 
              for s=numCyls+1 : length(P)
                  if sqrt( (P{s,1}.u(1)-x)^2 + (P{s,1}.u(2)-y)^2 + (P{s,1}.u(3)-z)^2 ) -2*r < 0
                     checkingSpheres = true; 
                  end
              end
          end
          s = bodySphere(m,r);  s.Fext = [0;0;-980*m;0;0;0];
          s.u = [x;y;z]; 
          s.body_type = 'sphere';
          %s.applyForce([0; 0; 0], [0; 0; -980*m; 0;0;0], 0, 1e8);
          %s.J = 2*m;
          %s.mu = 0; %%%%%%%%%%%%% FRICTIONLESS SPHERES %%%%%%%%%%%%%
          P = [P; {s}];  % Add sphere to the list of objects.
      end     
      Pspheres = {P{numCyls+1:end,1}};
      save(strcat('./box_of_spheres/box_of_sphers_',num2str(n),'.mat'),'Pspheres');
end 
sim.P = P;
sim.MAX_ITERS = 400;
sim.solver(solver_name);
sim.formulation(Formulation);
sim.COLLECT_DATA = 1;
%sim.RECORD_DATA = 1;
sim.STATIC_BODY = numCyls;
sim.run();
%data_set = sim.data_set;
%save('data_set.h5', 'data_set');
%save('slide_error.mat', 'num');
end