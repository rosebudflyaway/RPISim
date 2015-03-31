%-----
%This file can take the final joint angles for the powerball arm and move
%through a trajectory to reach the final position. q is the joint space
%trajectory
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

% sim.jointAngle = [80;0;0;90;0;0];
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

% Let's store all of our zero-position and quat offsets!
p1 = B1.u;
p2 = B2.u-B1.u;
p3 = B3.u-B2.u;
p4 = B4.u-B3.u;
p5 = B5.u-B4.u;
p6 = B6.u-B5.u;
p7 = B7.u-B6.u;

totaltime = 20;            %total timesteps for simulation
sim.jointAngle = [pi/2,pi/2,pi/6,pi/6,pi/6,pi/6];              %fianl joint angles
%TAngle=zeros([6,1]);
jaxis = zeros([6,3]);a1 = zeros([6,3]);
jaxis(1,:) = jnt1_axis;a1(1,:) = jnt1_axis
jaxis(2,:) = jnt2_axis;a1(2,:) = jnt2_axis;
jaxis(3,:) = jnt3_axis;a1(3,:) = jnt3_axis;
jaxis(4,:) = jnt4_axis;a1(4,:) = jnt4_axis;
jaxis(5,:) = jnt5_axis;a1(5,:) = jnt5_axis;
jaxis(6,:) = jnt6_axis;a1(6,:) = jnt6_axis;


q1 = [0,0,0,0,0,0];
q2 = sim.jointAngle;%[0,pi/4,0,pi/7,0,pi/8];
 q = mtraj(@lspb,q1,q2,[0:1:totaltime]);
TAngle = q';
for t = 1:totaltime
    %TAngle(:)=sim.jointAngle(:)*t/totaltime;
   
    % Rotation matrices
    R12 = rot( jaxis(1,:)', TAngle(1,t) ); 
    R23 = rot( jaxis(2,:)', TAngle(2,t) ); 
    R34 = rot( jaxis(3,:)', TAngle(3,t) ); 
    R45 = rot( jaxis(4,:)', TAngle(4,t) ); 
    R56 = rot( jaxis(5,:)', TAngle(5,t) ); 
    R67 = rot( jaxis(6,:)', TAngle(6,t) ); 
     
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
    B2.quat = qt( jaxis(1,:)', TAngle(1,t) ); 
    B3.quat = qtmultiply( B2.quat, qt( jaxis(2,:)', TAngle(2,t) ) ); 
    B4.quat = qtmultiply( B3.quat, qt( jaxis(3,:)', TAngle(3,t) ) ); 
    B5.quat = qtmultiply( B4.quat, qt( jaxis(4,:)', TAngle(4,t) ) ); 
    B6.quat = qtmultiply( B5.quat, qt( jaxis(5,:)', TAngle(5,t) ) ); 
    B7.quat = qtmultiply( B6.quat, qt( jaxis(6,:)', TAngle(6,t) ) ); 
    
    %Update axis
    jaxis(1,:) = (rot( jaxis(1,:)', TAngle(1,t))*a1(1,:)')'; 
    jaxis(2,:) = (rot( jaxis(2,:)', TAngle(2,t))*a1(2,:)')'; 
    jaxis(3,:) = (rot( jaxis(3,:)', TAngle(3,t))*a1(3,:)')';
    jaxis(4,:) = (rot( jaxis(4,:)', TAngle(4,t))*a1(4,:)')';
    jaxis(5,:) = (rot( jaxis(5,:)', TAngle(5,t))*a1(5,:)')';
    jaxis(6,:) = (rot( jaxis(6,:)', TAngle(6,t))*a1(6,:)')';


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
end
