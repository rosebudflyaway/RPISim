% test the penalty method, using a lot of spheres 
% Simulates a ~ [aa x bb x cc] "box" constructed from overlapping static cylinders.  
% Generates n spheres at various heights which fall into the box.
function [] = test_solvers()
% dimension of the box
aa = 0.67; 
bb = 0.432; 
cc = 0.25;
h = 0.28;  % height of cylinders % the thickness of the wall 
 
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
   density = 2.5e3;    
   r = 0.03;         % Radius of spheres.
   m = density*(4/3 * pi * r^3);  % Mass of spheres.
   %maxHeight = 0.15;  % Maximum height to places spheres.
   
   % the initial position
   xx = 0.1;    
   yy = 0.05; 
   zz = 0.05;
 
   s = obj_sphere(m, r);  
   s.Fext = [0; 0; -9.8*m; 0; 0; 0]; 
   s.static = 0;
   s.u = [xx; yy; zz];  
   P = [P; {s}];
   Pspheres = {s};
% Simulate    
   disp('This is the menu: ');
   fprintf('1: Lemke \t  2: PATH \t  3: fixed_point  \t  4: penalty  \t  5: mingres  \n');
   ch = input('Please enter the solver you will choose : ');
  
   switch(ch)
       case 1
           solver_name = 'Lemke';
           step_size = 1e-2;
       case 2
           solver_name = 'PATH';
           step_size = 1e-2;
       case 3
           solver_name = 'fixed_point';
           step_size = 1e-3;
       case 4
           solver_name = 'penalty';
           step_size = 1e-4;
       case 5
           solver_name = 'mingres';
           step_size = 1e-3;
       otherwise
           disp('Please refer to the menu for inputs'); 
           fprintf('1: Lemke \t  2: PATH \t  3: fixed_point  \t  4: penalty  \t  5: mingres  \n');
   end
   
   sim = Simulation(P, step_size);
   sim.SOLVER = solver_name;
 
 if (sim.RECORD_DATA == 0)
     sim.DRAW = 1;
     figure(1); hold on; axis equal; grid on;
     xlabel('X'); ylabel('Y'); zlabel('Z');
     % % view(3) sets the default three-dimensional view az = -37.5, e1 = 30
     view(3);
     % % Plot corners of box for reference, and box wire-frame
     plot3( [-aa/2 aa/2 aa/2 -aa/2 -aa/2 aa/2 aa/2 -aa/2], ...
         [-bb/2 -bb/2 bb/2 bb/2 -bb/2 -bb/2 bb/2 bb/2], ...
         [0 0 0 0 cc cc cc cc],'ro');
     plot3([-aa/2 aa/2 aa/2 -aa/2 -aa/2], [-bb/2 -bb/2 bb/2 bb/2 -bb/2], [0 0 0 0 0], 'r'); % Box bottom
     plot3([-aa/2 aa/2 aa/2 -aa/2 -aa/2], [-bb/2 -bb/2 bb/2 bb/2 -bb/2], [cc cc cc cc cc], 'g'); % Box top
     plot3([-aa/2 -aa/2],[-bb/2 -bb/2],[0 cc]);  % Four edges
     plot3([ aa/2  aa/2],[-bb/2 -bb/2],[0 cc]);
     plot3([ aa/2  aa/2],[ bb/2  bb/2],[0 cc]);
     plot3([-aa/2 -aa/2],[ bb/2  bb/2],[0 cc]);
 else
     sim.DRAW = 0;
 end
 
tic; 
count = 1;
% simulation start
%obj.STEP = 0;
%count_flag = true
while(1)
   for j = 1:count
        stat = 1;
        num_per_layer = 35;
        % there is some spheres which are far from collision on the floor
        if count > num_per_layer*3 && Pspheres{j}.u(3,1) - 7*r > 1e-2
           maxHeight = 0.15;
           stat = 0;
        end
        if count > num_per_layer*2 && Pspheres{j}.u(3,1) - 5*r > 1e-2
            stat = 0;
            maxHeight = 0.12;
        end
        if count > num_per_layer*1 && Pspheres{j}.u(3,1) - 3*r > 1e-2
            stat = 0; 
            maxHeight = 2.2*r;
        end
        if count < num_per_layer*1 && Pspheres{j}.u(3,1) - r > 1e-2
            stat =0;
            maxHeight = 1.2*r;
        end
   end
 
    if(stat == 1)
        checkingSpheres = true;
        while checkingSpheres
            checkingSpheres = false;
             x = -(aa/2-r) + ((aa/2-r)+(aa/2-r)) * rand(1);  
             y = -(bb/2-r) + ((bb/2-r)+(bb/2-r)) * rand(1);
             z = r + (maxHeight-r) * rand(1);  
            for s=numCyls+1:length(P)
                 if sqrt( (P{s,1}.u(1)-x)^2 + (P{s,1}.u(2)-y)^2 + (P{s,1}.u(3)-z)^2 ) -2*r < 0
                     checkingSpheres = true; 
                 end
            end   
        end
        s = obj_sphere(m, r);
        s.u = [x; y; z];  
        s.Fext = [0; 0; -9.8*m; 0; 0; 0]; 
        s.static = 0;
        obj.P{length(obj.P)+1, 1} = s;
        Pspheres{length(Pspheres)+1, 1} = s;    
        %sim = Simulation(P, step_size, obj.STEP, solver_name);
        count = count + 1;
        fprintf('There are %d spheres now \n', count);
    end
   
%     if(count == n)
%         radcyl = 2.9e-3;
%         hcyl = 0.25;
%         mass = pi*radcyl*radcyl*hcyl*density;
%         cyl = obj_cylinder(1,radcyl,hcyl); cyl.static=0; cyl.u=[0;0;0.35];  
%         cyl.Fext = [0; 0; -9.8*mass; 0; 0; -1];
%         P = [{b1}; {b2}; {b3}; {b4}; {b5}; {cyl}; Pspheres];
%         sim = Simulation(P, 0.01);
%         count = count + 1;
%     end
% Simulate    
    
    obj = sim.step();
%     for m = 1:count
%         if(Pspheres{m,1}.u(1,1) > aa/2 || Pspheres{m,1}.u(1,1) < -aa/2 || Pspheres{m,1}.u(2,1) > bb/2 || Pspheres{m,1}.u(2,1) <-bb/2 || Pspheres{m,1}.u(3,1) < 0 )
%             break;
%         end
%     end
    for i=1:length(obj.P)
        if obj.P{i}.static == 0 && obj.P{i}.u(3,1) > 0.14
            obj.P{i}.nu(1:3) = [0; 0; 0];
        end
    end
    
%     if(count > 80) 
%         break;
%     end
end % end while
% fclose(fid);
for i=1:30
    obj = sim.step();
end

fprintf('The number of spheres is : %d', count);
total_time = toc;
disp(total_time);
end