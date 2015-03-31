
function sim = snake_joints(numbodies, jointCorrection, max_iters, gui)

    mass = 0.5; 
    radius = 0.3;
    length = 3; 
    spacing = 0.2; 

    %numbodies = 4; 
   

    % First body of snake
    body1 = mesh_cylinder(5,mass,radius,length); 
        body1.quat = qt([1;0;0],pi/2);
        body1.Fext = [0;0;0;0;0;0]; 
        body1.color = [1 0 0]; 
    bodies = {body1}; 

    % Add more bodies
    for i=1:numbodies-1
       body = mesh_cylinder(5,mass,radius,length);
       body.quat = qt([1;0;0], pi/2); 
       body.u = [0; i*(length+spacing); 0];
       body.Fext = [0;0;0;0;0;0]; 
       bodies = [bodies; {body}];
    end

    % Add a Fext to the first body
    bodies{1}.Fext(6) = .5;

    % Init simulator
    sim = Simulation(bodies,0.01);
    sim.setFriction(false); 

    % Add a joints
    dir = [0;0;1];  

    DIR = rand(3,numbodies);
    for i=1:numbodies-1
%        sim.addJoint(bodies{i},bodies{i+1},[0;i*(length+spacing)-(length+spacing)/2;0;],dir,'revolute'); 

        %sim.addJoint(bodies{i},bodies{i+1},[0;i*(length+spacing)-(length+spacing)/2;0;],DIR(1:3,i),'spherical');
        sim.addJoint(bodies{i},bodies{i+1},[0;i*(length+spacing)-(length+spacing)/2;0;],dir,'spherical');
    end
    sim.MAX_ITERS = max_iters; 
    sim.GUI.DRAW = gui; 
    sim.jointCorrection = jointCorrection; 
    sim.run();





