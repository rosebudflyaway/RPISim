%-----
%This file can take the final joint angles for the powerball arm and
%achieve them using robocontroller. It tries to do it in 200 time steps
%it does not have collision detection switched on
%-----

%creat objects
load('powerballmesh.mat')

hand = mesh_cylinder(4,1,0.02,0.3);
hand.quat = qt([1;0;0],pi/2);
hand.u = [0;0.1;0.92];
hand.nu = [0;0;0;0;0;1];

obj1 = mesh_cylinder(3,1,0.01,0.2); 
obj1.u = [ -0.07;0.15;0.90];
obj1.nu = [0;0;0;0;0;0];
%end creating bodies

bodies = { erb1451;erb1452;erb115;larm;arm1;base;hand};
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

%define which bodies dont collide

% base.doNotCollideWith(erb1451.bodyID);
% base.doNotCollideWith(erb1452.bodyID);
% base.doNotCollideWith(erb115.bodyID);
% base.doNotCollideWith(larm.bodyID);
% base.doNotCollideWith(arm1.bodyID);
% erb1451.doNotCollideWith(arm1.bodyID);
% erb1451.doNotCollideWith(base.bodyID);
% erb1451.doNotCollideWith(erb1452.bodyID);
% erb1451.doNotCollideWith(erb115.bodyID);
% erb1451.doNotCollideWith(larm.bodyID);
% erb1452.doNotCollideWith(base.bodyID);
% erb1452.doNotCollideWith(arm1.bodyID);
% erb1452.doNotCollideWith(larm.bodyID);
% erb1452.doNotCollideWith(erb1451.bodyID);
% erb1452.doNotCollideWith(erb115.bodyID);
% erb115.doNotCollideWith(base.bodyID);
% erb115.doNotCollideWith(erb1451.bodyID);
% erb115.doNotCollideWith(larm.bodyID);
% erb115.doNotCollideWith(arm1.bodyID);
% erb115.doNotCollideWith(erb1452.bodyID);
% larm.doNotCollideWith(base.bodyID);
% larm.doNotCollideWith(erb1451.bodyID);
% larm.doNotCollideWith(erb1452.bodyID);
% larm.doNotCollideWith(erb115.bodyID);
% larm.doNotCollideWith(arm1.bodyID);
% arm1.doNotCollideWith(base.bodyID);
% arm1.doNotCollideWith(erb1451.bodyID);
% arm1.doNotCollideWith(erb1452.bodyID);
% arm1.doNotCollideWith(larm.bodyID);
% arm1.doNotCollideWith(erb115.bodyID);
% 
% hand.doNotCollideWith(base.bodyID);
% hand.doNotCollideWith(erb1451.bodyID);
% hand.doNotCollideWith(arm1.bodyID);
% hand.doNotCollideWith(erb1452.bodyID);
% hand.doNotCollideWith(larm.bodyID);
% larm.doNotCollideWith(hand.bodyID);
% erb1452.doNotCollideWith(hand.bodyID);
% arm1.doNotCollideWith(hand.bodyID);
% erb1451.doNotCollideWith(hand.bodyID);
% base.doNotCollideWith(hand.bodyID);
% 
% obj1.doNotCollideWith(base.bodyID);
% obj1.doNotCollideWith(erb1451.bodyID);
% obj1.doNotCollideWith(arm1.bodyID);
% obj1.doNotCollideWith(erb1452.bodyID);
% obj1.doNotCollideWith(larm.bodyID);
% obj1.doNotCollideWith(erb115.bodyID);
% erb115.doNotCollideWith(obj1.bodyID);
% larm.doNotCollideWith(obj1.bodyID);
% erb1452.doNotCollideWith(obj1.bodyID);
% arm1.doNotCollideWith(obj1.bodyID);
% erb1451.doNotCollideWith(obj1.bodyID);
% base.doNotCollideWith(obj1.bodyID);

sim.jointAngle = [1;1;1;1;1;1];
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

<<<<<<< .mine
%sim.setRecord(true);
%sim.disableGUI(); 
sim.MAX_ITERS = 1000; 
=======
%sim.setRecord(true);
%sim.disableGUI(); 
%sim.MAX_ITERS = 1000; 
>>>>>>> .r393

sim.run;



