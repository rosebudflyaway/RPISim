  
 % test the penalty method, using a lot of spheres 

% Simulates a ~ [aa x bb x cc] "box" constructed from overlapping static cylinders.  
% Generates n spheres at various heights which fall into the box.
function [] = test_layers()
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
           solver_name = 'fix_point';
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
 %if (sim.RECORD_DATA == 0)
   %  sim.DRAW = 1;
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
 %else
 %    sim.DRAW = 0;
% end
 
 xnum = floor((aa-4*r)/(2.2*r)) + 2;
 ynum = floor((bb-4*r)/(2.2*r)) + 2;
 xcenter = zeros(xnum, 1);
 ycenter = zeros(ynum, 1);
 for i = 1:xnum
     xcenter(i) = (-aa/2+2*r) + (i-1)*2*r;
 end
 
 for j = 1:ynum
     ycenter(j) = (-bb/2+2*r)  + (j-1)*2*r;
 end
 
 
tic; 
count = 1;
oddflag = 1;
z = 0.16;
while(count < 140)
    for i = 1: xnum;
        for j = 1 : ynum;
            if (oddflag == 1  && mod((i+j), 4) == 2)
                s = obj_sphere(m, r);
                s.u = [xcenter(i); ycenter(j); z];
%                 s.nu = [0.1*(-1+rand(1)*2); 0.1*(-1+rand(1)*2); 0; 0; -1; 0];
                s.nu = [(-1+rand(1)*2); (-1+rand(1)*2); 0; 0; -1; 0];
                s.Fext = [0; 0; -9.8*m; 0; 0; 0];
                s.static = 0;
                P = [P; {s}];
                %sim = Simulation(P, step_size, obj.STEP, solver_name);
                count = count + 1;
                fprintf('There are %d spheres now \n', count);
            end
            if(oddflag == -1 && mod((i+j), 4) == 0)
                s = obj_sphere(m, r);
                s.u = [xcenter(i); ycenter(j); z];
                s.Fext = [0; 0; -9.8*m; 0; 0; 0];
                s.static = 0;
                P = [P; {s}];
                %sim = Simulation(P, step_size, obj.STEP, solver_name);
                count = count + 1;
                fprintf('There are %d spheres now \n', count);
            end
            
        end
    end
    oddflag = -1 * oddflag;
    sim = Simulation(P, step_size);
    sim.SOLVER = solver_name;
    for k = 1 : 20
        obj = sim.step();
    end
    for i=1:length(obj.P)
        if obj.P{i}.static == 0 && obj.P{i}.u(3,1) > 0.14
            obj.P{i}.nu(1:3) = [0; 0; 0];
        end
    end
    
%      if(count > 110) 
%         break;
%      end
end % end while
% fclose(fid);
fprintf('The number of spheres is : %d', count);
total_time = toc;
disp(total_time);
end