  
 % test the penalty method, using a lot of spheres 

% Simulates a ~ [aa x bb x cc] "box" constructed from overlapping static cylinders.  
% Generates n spheres at various heights which fall into the box.
function [] = save_base(n)
% dimension of the box
% aa = 87; 
% bb = 53.2; 
% cc = 45;
% h = 28;  % height of cylinders % the thickness of the wall 

aa = 46; 
bb = 26; 
cc = 20;
h = 10;  % height of cylinders % the thickness of the wall 
 
% Box
% Bottom  (mass, radius, height) 
  rb = 0.5*sqrt(aa*aa + bb*bb) + 0.5*min(aa, bb);
  b1 = obj_cylinder(1,rb,h); b1.static = 1; b1.u=[0;0;-h/2];  

% Sides
% yz plane 
  ryz = 0.5*sqrt(bb*bb+cc*cc) + 0.5*min(bb, cc);
  b2 = obj_cylinder(1,ryz,h); b2.static = 1; b2.u=[-(aa/2+h/2);0;cc/2]; b2.quat = quat([0;1;0],pi/2);
  b3 = obj_cylinder(1,ryz,h); b3.static = 1; b3.u=[ (aa/2+h/2);0;cc/2]; b3.quat = quat([0;1;0],pi/2);
% xz plane 
  rxz = 0.5*sqrt(aa*aa+cc*cc) + 0.5*min(aa, cc);
  b4 = obj_cylinder(1,rxz,h); b4.static = 1; b4.u=[0;-(bb/2+h/2);cc/2]; b4.quat = quatmultiply(quat([0;1;0],pi/2),quat([0;0;1],pi/2));
  b5 = obj_cylinder(1,rxz,h); b5.static = 1; b5.u=[0; (bb/2+h/2);cc/2]; b5.quat = quatmultiply(quat([0;1;0],pi/2),quat([0;0;1],pi/2));
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
   fprintf('1: Lemke \t  2: PATH \t  3: fixed_point  \t  4: penalty  \t  5: mingres \t 6: Fischer-Newton \n');
   ch = input('Please enter the solver you will choose : ');
  
   switch(ch)
       case 1
           solver_name = 'Lemke';
           step_size = 0.01;
       case 2
           solver_name = 'PATH';
           step_size = 1e-2;
       case 3
           solver_name = 'fixed_point';
           step_size = 1e-3;
       case 4
           solver_name = 'penalty';
           step_size = 1e-3;
       case 5
           solver_name = 'mingres';  
           step_size = 1e-3;
       case 6
           solver_name = 'fischer_newton';
           step_size = 1e-4;
       otherwise
           disp('Please refer to the menu for inputs'); 
           fprintf('1: Lemke\t 2: PATH \t 3: fixed_point\t 4: penalty\t 5: mingres\t 6: Fischer-Newton\n');
   end
   disp(solver_name);
   sim = Simulation(P, step_size);
   sim.SOLVER = solver_name;
 
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
      m = 200;           % Mass to use for spheres.
      r = 1.5;         % Radius to use for spheres. 
      maxHeight = 15;  % Maximum height to places spheres.
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
          s = obj_sphere(m,r);  s.Fext = [0;0;-980*m;0;0;0];
          s.u = [x;y;z]; 
          %s.applyForce([0; 0; 0], [0; 0; -980*m; 0;0;0], 0, 1e8);
          %s.J = 2*m;
          %s.mu = 0; %%%%%%%%%%%%% FRICTIONLESS SPHERES %%%%%%%%%%%%%
          P = [P; {s}];  % Add sphere to the list of objects.
      end     
      Pspheres = {P{numCyls+1:end,1}};
      save(strcat('./box_of_spheres/box_of_sphers_',num2str(n),'.mat'),'Pspheres');
end 
 
r = 2.5;
m = 20;
if (n==100)
    drop_time = 30*step_size;
    touch_time = 43*step_size;
end

if(n==200)
    drop_time = 30*step_size;
    touch_time = 47*step_size;
end
posi = [-aa/2+3*r; 0+1*r; cc];        
shoe = obj_union(2, 2.5, 6, posi);        
% stable period
shoe.applyForce([0; 0; 0], [0; 0; 0], 0, drop_time);
% dropping period; apply gravity at the center of mass
shoe.applyForce([0; 0; 0], [0; 0; -980*m], drop_time, 1e6);
%shoe.applyForce([3*r; r; 0], [1000; 0; -980*m], touch_time, 1e6);
%shoe.applyForce([0; 0; 0], [0; 0; -980*m], touch_time, 1e6);

%apply the horizontal force at the front of the foot 
shoe.applyForce([3*r; 0; 0], [60000; 0; 0], touch_time, 1e6);
shoe.applyForce([0; 0; 0], [0; 0; -10000], touch_time, 1e6);

 t0 = touch_time - step_size; 
 tf = touch_time + step_size;
 shoe.Reset_velocity([0; 0; 0; 0; 0; 0], t0, tf);

P = [P; {shoe}];
sim.P = P;
%sim.DRAW = 1; 
sim.MAX_ITERS = 80; 
% figure(1); hold on; axis equal; grid on;
% xlabel('X'); ylabel('Y'); zlabel('Z');
% view(3);

%start = cputime;
%data = zeros(12, 1400);
%sim.MAX_ITERS = 30;
sim.run();
end