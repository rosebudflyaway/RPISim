
% Simulates a ~ [1 x 1/2 x 1/4] "box" constructed from overlapping static cylinders.
% Generates n spheres at various heights which fall into the box.
function [conv] = test_box_mingres(n)
% Box
% Bottom  (mass, radius, height)
h = 0.05;  % height of cylinders
aa = 0.4;
bb = 0.36;
cc = 0.24;
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
% Here, we look for a file containing the positions of n spheres.  If it
% doesn't exist, then we'll create it.  We do this in order to generate a
% random scene that can be re-used.
density = 1.06e3;
r = 2.905e-2;         % Radius to use for spheres.
m = density*(4/3 * pi * r^3);  % Mass to use for spheres.
maxHeight = 0.6;  % Maximum height to places spheres.

if exist( strcat('box_of_sphers_',num2str(n),'.mat'), 'var')
    disp(['Note: a file was found for ' num2str(n) ' spheres and will be used.']);
    Pspheres = load ( strcat('box_of_sphers_',num2str(n),'.mat') );
    P = [P; Pspheres'];
else
    disp(['Randomly placing ' num2str(n) ' spheres']);
    
    for i=1:n
        % Ensure that we don't create overlapping spheres
        checkingSpheres = true;
        while checkingSpheres
            checkingSpheres = false;
            % Generate random x,y,z
            x = -(aa/2-r-h) + (aa- 2*r-h) * rand(1);
            y = -(bb/2-r-h) + (bb-2*r-h) * rand(1);
            z = r + (maxHeight) * rand(1);
            % right now to get the convergent error, I will not draw !!!!!!!!!!!!!!!!!!!!!!change back if see animation
            % Check position against all other spheres.
            for s=numCyls+1 : length(P)
                % if in collision initially
                if sqrt( (P{s,1}.u(1)-x)^2 + (P{s,1}.u(2)-y)^2 + (P{s,1}.u(3)-z)^2 ) -2*r < 0
                    checkingSpheres = true;
                end
            end
        end
        
        s = obj_sphere(m,r);  s.Fext = [0;0;-9.8*m;0;0;0];
        s.u = [x;y;z];
        %s.mu = 0; %%%%%%%%%%%%% FRICTIONLESS SPHERES %%%%%%%%%%%%%
        P = [P; {s}];  % Add sphere to the list of objects.
        
    end
    
    Pspheres = P{numCyls+1:end,1};
    % save workspace variables to file
    save(strcat('boxgetframe_of_sphers_',num2str(n),'.mat'),'Pspheres');
end
% Simulate
%disp('before calling the simulation');
sim = Simulation(P,.01, 0, 'mingres');
%disp('after calling the simulation');
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
% fid = fopen('PATH.dat', 'w+');
% conv = zeros(150, 1);

tic;

for i = 1:50
    fprintf('This is the %d iteration  before \n', i);
    obj = sim.step();
    fprintf('This is the %d iteration  after \n', i);
    %disp(obj.time);
    % fprintf(fid, '%f \n', obj.error);
    %M(:,i) = getframe();
    %conv(i, 1) = obj.error;
end
% fclose(fid);
end


