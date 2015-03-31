

%function sim = hanging_joint(jointCorrection, max_iters, gui)
 jointCorrection = true;
 max_iters = 1000;
  gui = true;
% Create 2 bodies
body1 = bodyCylinder(1,1,20);
    body1.u = [0;0;2];
    body1.quat = qt([1;0;0],pi/2);
    body1.static = true;
    body1.color = [.6 .6 .6]; 

body2 = bodyCylinder(2,1,8); 
    body2.u = [ 0;0;-4];
    body2.quat = qt([1;0;0],.2);  body2.u(2)=.7947;
    %body2.Fext(4) = .1;

  
% Init simulator
bodies = {body1; body2};
sim = Simulation(bodies,0.01);
sim.setFriction(false); 

% Add a joint
jnt_pos = [0; 0; 0];
jnt_axis = [0; 0; 1]; 
%jnt_axis = rand(3,1); 
%jnt_axis = [0.5714; 0.5991; 0.5609]; 
sim.addJoint(body2,body1,jnt_pos,jnt_axis,'spherical'); 

sim.jointCorrection = jointCorrection;
sim.MAX_ITERS = max_iters;
sim.GUI.DRAW = gui; 

sim.run();



