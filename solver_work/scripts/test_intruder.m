 
% test the penalty method, using a lot of spheres 

% Simulates a ~ [aa x bb x cc] "box" constructed from overlapping static cylinders.  
% Generates n spheres at various heights which fall into the box.
function [] = test_intruder(n)
% dimension of the box
aa = 0.5; 
bb = 0.36; 
cc = 0.34;
h = 0.05;  % height of cylinders % the thickness of the wall 
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
   density = 1.06e6;    
   r = 2.905e-2;         % Radius to use for spheres.
   m = density*(4/3 * pi * r^3);  % Mass to use for spheres.
   maxHeight = 0.15;  % Maximum height to places spheres.
   % the initial position
   xx = 0.1;    
   yy = 0.05; 
   zz = 0.24;
 
   s = obj_sphere(m, r);  
   s.Fext = [0; 0; -9.8*m; 0; 0; 0]; 
   s.u = [xx; yy; zz];  
   P = [P; {s}];
   Pspheres = {s};
% Simulate    
 
 solver_name = 'Lemke';
 sim = Simulation(P, 1e-3,0,solver_name); 
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
tic; 
count = 1;
% simulation start
obj.STEP = 0;
 radcyl = 8e-3;       
 hcyl = 0.25;
count_flag = true;
spherebf = 1;
for i = 1:150000
   
    for j = 1:count
        stat = 1;
        % there is some sphere which is far from collision on the floor
        if(Pspheres{j}.u(3,1) - 5*r > 1e-2)
           stat = 0;
        end
    end  
 
    if(stat == 1 && count < n)
        checkingSpheres = true;
        while checkingSpheres
            checkingSpheres = false;
            x = -(aa/2-r-h) + (aa- 2*r-h) * rand(1);  
            y = -(bb/2-r-h) + (bb-2*r-h) * rand(1);
            z = r + (maxHeight) * rand(1);   
            for s=numCyls+1:length(P)
                 if sqrt( (P{s,1}.u(1)-x)^2 + (P{s,1}.u(2)-y)^2 + (P{s,1}.u(3)-z)^2 ) -2*r < 0
                     checkingSpheres = true; 
                 end
            end
        end
        s = obj_sphere(m, r);
        s.u = [x; y; z];  
        s.Fext = [0; 0; -9.8*m; 0; 0; 0]; 
        P = [P; {s}];
        Pspheres = [Pspheres; {s}];
        sim = Simulation(P, 0.01, obj.STEP, solver_name);
        count = count + 1;
        fprintf('There are %d spheres now \n', count);
    end
   
    if(count == n && spherebf)
        spherebf = 0;
        density = 1e4;
        mass = pi*radcyl*radcyl*hcyl*density;
        cyl = obj_cylinder(1,radcyl,hcyl); cyl.u=[0;0;0.25];  
        cyl.Fext = [0; 0; -9.8*mass; 0; 0; -1];
        P = [{b1}; {b2}; {b3}; {b4}; {b5}; {cyl}; Pspheres];
        sim = Simulation(P, 0.01, obj.STEP, solver_name);       
        %count = count + 1;
        
    end
    
       
       
% Simulate    
    obj = sim.step();
%      disp(size(P));
     
%      if(length(P) > 15)
%          disp(P{6}.body_type);
%          pause;
%      end
%     
%         disp('This is the position of the cylinder');
%         disp(P{6}.u);
%         pause;
    %disp(obj.time);
   % fprintf(fid, '%f \n', obj.error);
    %M(:,i) = getframe();
    %conv(i, 1) = obj.error;
    
    % This is to memorize the steps when the number reached the maximum 
    % to put down the temp_step;
    
    if(count == n && count_flag)
        count_flag = false;
        temp_step = obj.STEP;
        %disp(temp_step);
    end
    if(count == n && (obj.STEP-temp_step > 200))
        disp('The number of step has reached the threshhold');
        disp(obj.STEP);
        %pause;
        break;
    end
    
    % If the intruder will collide with the bottom plane 
    if((P{6}.u(3,1) - 0.5*hcyl < 1e-2) && strcmp(P{6}.body_type, 'cylinder'))
        disp('The cylinder has hit the bottom of the box');
        disp(obj.STEP);
        break;
    end
end
% fclose(fid);
total_time = toc;
disp(total_time);
end


