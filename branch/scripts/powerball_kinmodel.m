%-----
%This file can take the final joint angles for the powerball arm and give
%the final position of the arm
%-----

%creat objects
load('powerballmesh.mat')


hand = mesh_cylinder(4,1,0.02,0.3);
quat = qt([1;0;0],pi/2);
hand.u = [0;0.1;0.92];
hand.nu = [0;0;0;0;0;1];
for v = 1:hand.num_verts
       hand.verts(v).local_coords = qtrotate(quat,hand.verts(v).local_coords); 
end
obj1 = mesh_cylinder(3,1,0.01,0.2); 
obj1.u = [ -0.07;0.15;0.90];
obj1.nu = [0;0;0;0;0;0];
%end creating bodies

bodies = { base;erb1451;arm1;erb1452;larm;erb115;hand};
sim = Simulation(bodies, 0.01);

hand.static = false;
jnt6_pos = [0; 0; .860];
jnt6_axis = [0;0 ; 1];

erb115.static = false;
%erb115.nu = [0;0;0;0;0;0]; 
jnt5_pos = [0; 0; .860];
jnt5_axis = [1;0 ; 0]; 

larm.static = false;
%larm.nu = [0;0;0;0;0;3];
jnt4_pos = [0;0;.555];
jnt4_axis = [0;0;1];

erb1452.static = false;
%erb1452.nu = [0;0;0;1;0;0];
jnt3_pos = [0;0;.555];
jnt3_axis = [1;0;0];

arm1.static = false;
jnt2_pos = [0;0;.205];
jnt2_axis = [1;0;0];

erb1451.static = false;
%erb1451.nu = [0;0;0;0;0;1];
jnt1_pos = [0;0;.205];
jnt1_axis = [0;0;1];



sim.addJoint(hand,erb115,jnt6_pos,jnt6_axis,'revolute');
sim.addJoint(erb115,larm,jnt5_pos,jnt5_axis,'revolute'); 
sim.addJoint(larm,erb1452,jnt4_pos,jnt4_axis,'revolute');
sim.addJoint(erb1452,arm1,jnt3_pos,jnt3_axis,'revolute');
sim.addJoint(arm1,erb1451,jnt2_pos,jnt2_axis,'revolute');
sim.addJoint(erb1451,base,jnt1_pos,jnt1_axis,'revolute');
%sim.COLLISIONS = true;
sim.COLLISIONS = false;
sim.gravityOFF();
%sim.jointCorrection = false; 
sim.FRICTION = false; 
sim.controller( 'roboController' );
%sim.setRecord(true);
%sim.disableGUI(); 
sim.MAX_ITERS = 0; 

sim.run;

jointAngle = [90;10;45;45;25;45];
% theta= zeros([3,4]);
% theta(1,:) = (qt([0;0;1],jointAngle(1)*pi/180))';
% theta(2,:) = (qt([1;0;0],jointAngle(2)*pi/180))';
% theta(3,:) = (qt([1;0;0],jointAngle(3)*pi/180))';
%theta(4,:) = (qt([0;0;1],jointAngle(4)*pi/180))';
%theta(5,:) = (qt([1;0;0],jointAngle(5)*pi/180))';
%theta(6,:) = (qt([0;0;1],jointAngle(6)*pi/180))';


% Primary axes
X = [1;0;0];
Y = [0;1;0];
Z = [0;0;1];

% Easier body names
B1 = base; 
B2 = erb1451;
B3 = arm1;
B4 = erb1452;
B5 = larm; 
B6 = erb115;
B7 = hand;

% Let's store all of our zero-position offsets!
p1 = B1.u;
p2 = B2.u-B1.u;
p3 = B3.u-B2.u;
p4 = B4.u-B3.u;
p5 = B5.u-B4.u;
p6 = B6.u-B5.u;
p7 = B7.u-B6.u;

% Rotation matrices
R12 = rot( jnt1_axis, jointAngle(1)*pi/180 ); 
R23 = rot( jnt2_axis, jointAngle(2)*pi/180 ); 
R34 = rot( jnt3_axis, jointAngle(3)*pi/180 ); 
R45 = rot( jnt4_axis, jointAngle(4)*pi/180 ); 
R56 = rot( jnt5_axis, jointAngle(5)*pi/180 ); 
R67 = rot( jnt6_axis, jointAngle(6)*pi/180 ); 
  
% Update positions
B1.u = p1;                % Zero offset
B2.u = p1 + R12*p2;
B3.u = p1 + R12*p2 + R12*R23*p3;
B4.u = p1 + R12*p2 + R12*R23*p3 + R12*R23*p4;  % NO R34!
B5.u = p1 + R12*p2 + R12*R23*p3 + R12*R23*p4 + R12*R23*R34*R45*p5;
B6.u = p1 + R12*p2 + R12*R23*p3 + R12*R23*p4 + R12*R23*R34*R45*p5 + R12*R23*R34*R45*p6; % NO R56!
B7.u = p1 + R12*p2 + R12*R23*p3 + R12*R23*p4 + R12*R23*R34*R45*p5 + R12*R23*R34*R45*p6 + R12*R23*R34*R45*R56*R67*p7;

% Update rotations
B1.quat = [1;0;0;0]; 
B2.quat = qt( jnt1_axis, jointAngle(1)*pi/180 ); 
B3.quat = qtmultiply( B2.quat, qt( jnt2_axis, jointAngle(2)*pi/180 ) ); 
B4.quat = qtmultiply( B3.quat, qt( jnt3_axis, jointAngle(3)*pi/180 ) ); 
B5.quat = qtmultiply( B4.quat, qt( jnt4_axis, jointAngle(4)*pi/180 ) ); 
B6.quat = qtmultiply( B5.quat, qt( jnt5_axis, jointAngle(5)*pi/180 ) ); 
B7.quat = qtmultiply( B6.quat, qt( jnt6_axis, jointAngle(6)*pi/180 ) ); 


% Update graphics
B1.update_world_position(); B1.draw();
B2.update_world_position(); B2.draw();
B3.update_world_position(); B3.draw();
B4.update_world_position(); B4.draw(); 
B5.update_world_position(); B5.draw(); 
B6.update_world_position(); B6.draw(); 
B7.update_world_position(); B7.draw(); 
drawnow; 
axis equal; 





% i = length(bodies);
% while(i>0)
%     j=i;
%     p=0;
%     r = sim.P{i}.quat;
%     while(j>1)
%         p = p + qt2rot((theta(j-1,:))')*(sim.P{j}.u-sim.P{j-1}.u);
% %             sim.P{j}.u-sim.P{j-1}.u
% %             qt2rot((theta(j-1,:))')*(sim.P{j}.u-sim.P{j-1}.u)
% %             qt2rot((theta(j-1,:))')
%         r = qtmultiply(r,theta(j-1,:)');
%         j=j-1;
%     end
%     sim.P{i}.u=p;
%     sim.P{i}.quat = r;
%     t=r;
%     sim.P{i}.update_world_position();
%     sim.P{i}.draw;
%     i=i-1; 
% end

