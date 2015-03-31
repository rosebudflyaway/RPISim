  
 % test the penalty method, using a lot of spheres 

% Simulates a ~ [aa x bb x cc] "box" constructed from overlapping static cylinders.  
% Generates n spheres at various heights which fall into the box.
function [] = test_fix_point(n)
% dimension of the box
aa = 0.67; 
bb = 0.432; 
cc = 0.25;
h = 0.28;  % height of cylinders % the thickness of the wall 
RECORD_DATA = 0;
DATA_DIRECTORY = ['sim_data_fix_point' datestr(now, 'mm_dd_yy_HH_MM_SS')];
disp(DATA_DIRECTORY);

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
   r = 0.03;         % Radius to use for spheres.
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
 
 solver_name = 'fix_point';
 sim = Simulation(P, 1e-3,0,solver_name);
 
 if (RECORD_DATA == 0)
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
obj.STEP = 0;
count_flag = true;
for i = 1:1500000
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
        P = [P; {s}];
        Pspheres = [Pspheres; {s}];
        sim = Simulation(P,3e-5, obj.STEP, solver_name);
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
    sim.DRAW = 1;
    obj = sim.step();
    %disp(obj.time);
   % fprintf(fid, '%f \n', obj.error);
    %M(:,i) = getframe();
    %conv(i, 1) = obj.error;
    
    if(RECORD_DATA == 1)
        % create folder at the first step
        if obj.STEP == 1
            mkdir (DATA_DIRECTORY);
            % P = obj.P;
            %save([DATA_DIRECTORY '/bodies_1.mat'], 'P');
            DATA_fileID = fopen([DATA_DIRECTORY '/sim_log.txt'], 'w');
        end
        
        fprintf(DATA_fileID, '%f,  ', obj.TIME);
        fprintf(DATA_fileID, '%d, ', length(P));
        
        for k = 1 : length(obj.P)
            fprintf(DATA_fileID,'%f,%f,%f,%f,%f,%f,%f,',obj.P{k}.u(1),obj.P{k}.u(2),obj.P{k}.u(3),obj.P{k}.quat(1),obj.P{k}.quat(2),obj.P{k}.quat(3),obj.P{k}.quat(4));
        end
        fprintf(DATA_fileID, '\n');
    end  % if(RECORD_DATA = 1)
    
    
    if(count == n && count_flag)
        % save the total n bodies information
        if(RECORD_DATA == 1)
        save([DATA_DIRECTORY '/bodies.mat'], 'P');
        end
        count_flag = false;
        temp_step = obj.STEP;
        %disp(temp_step);
    end
    if(count == n && (obj.STEP-temp_step > 10000))
        disp(obj.STEP);
        %pause;  
        break;
    end
end
% fclose(fid);
total_time = toc;
disp(total_time);
end


