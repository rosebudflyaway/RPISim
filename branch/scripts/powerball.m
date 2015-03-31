
% First attempt at an approximate powerball arm

% Define all the bodies
base = mesh_cylinder(10, 1, 0.07, .160);
    base.u = [0;0;.16/2];
    base.setStatic( true );

s1 = bodySphere(1, 0.06);
    s1.u = [0;0;.205];  s1.Fext(6) = 1;
    
c1 = mesh_cylinder(10, 1, 0.03, .370); 
    c1.u = [.080; 0; .350];

s2 = bodySphere(1, 0.06);
    s2.u = [0; 0; 0.555];

c2 = mesh_cylinder(10, 1, 0.03, .290); 
    c2.u = [.070; 0; .710];

s3 = bodySphere(1, 0.04); 
    s3.u = [0; 0; .860];
    
bodies = { base
           s1
           c1
           s2
           c2
           s3 };
       
% Initialize simulator
Sim = Simulation(bodies, 0.01); 
Sim.setFriction(false);
Sim.COLLISIONS = false; 
Sim.jointCorrection = false; 

% Create joints
X = [1;0;0];
Y = [0;1;0];
Z = [0;0;1];
Sim.addJoint( base, s1, s1.u, Z, 'revolute');
Sim.addJoint( s1, c1, s1.u, X, 'revolute');

%Sim.gravityOFF(); 
Sim.run();
       


