
% Script that creates a robotic arm

% Create arm components
Base0 = mesh_cylinder(5, 1, 0.1, 0.04);
    Base0.setStatic( true ); 

Base = mesh_cylinder(10, 1, 0.2, 0.02);
    Base.u = [0;0;-.01];
    %Base.setStatic( true );
    
arm1 = mesh_cylinder(10, 1, 0.05, 0.5); 
    arm1.u = [0; -.25; 0.0]; 
    arm1.quat = qt([1;0;0],pi/2);
    %arm1.Fext(4) = -1;
    %arm1.setStatic( true ); 
    
arm2 = mesh_cylinder(10, 1, 0.04, 0.4); 
    arm2.color = [1 0 0]; 
    arm2.quat = qt([1;0;0],pi/2);
    arm2.u = [0; -.5 - .2; 0];
    arm2.Aext(4) = -5;

% Initialize simulator
bodies = { Base0
           Base 
           arm1 
           arm2 };       
Sim = Simulation(bodies, 0.01); 

% Add joints 
Sim.addJoint( Base0, Base, [0;0;0],[0;0;1],'revolute'); 
Sim.addJoint( Base, arm1, [0;0;0],[1;0;0],'revolute');
Sim.addJoint( arm1, arm2, [0;-.5;0],[1;0;0],'revolute'); 

% Don't do collision detection
Sim.COLLISIONS = false; 

%Sim.jointCorrection = false; 
Sim.controller( 'roboController' ); % Arm controller

Sim.FRICTION = false;
%Sim.jointCorrection = true; 
%Sim.gravityON();
Sim.run();     

