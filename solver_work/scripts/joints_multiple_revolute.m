

mass = 0.5; 
radius = 0.3;
length = 3; 
spacing = 0.2; 

numbodies = 3; 

% First body of snake
body1 = bodyCylinder(mass,radius,length); 
    body1.quat = quat([1;0;0],pi/2);
    body1.Fext = [0;0;0;0;0;0]; 
bodies = {body1}; 

% Add more bodies
for i=1:numbodies-1
   body = bodyCylinder(mass,radius,length);
   body.quat = quat([1;0;0], pi/2); 
   body.u = [0; i*(length+spacing); 0];
   body.Fext = [0;0;0;0;0;0]; 
   bodies = [bodies; {body}];
end

% Add a velocity to the middle body
%bodies{floor(size(bodies,1)/2)}.Fext(1) = 0.2;
bodies{1}.Fext(1) = 0.1;
    
% Init simulator
sim = Simulation(bodies,0.01);
sim.setFriction(false); 
%sim.gravityOFF(); 

% Add a joints
dir = [0;0;1];  

for i=1:numbodies-1
   sim.addJoint(bodies{i},bodies{i+1},[0;i*(length+spacing)-(length+spacing)/2;0;],dir,'revolute'); 
end

% Run simulator
% sim.setRecord(true);
% sim.disableGUI;
% sim.MAX_ITERS = 3000; 
sim.run();



