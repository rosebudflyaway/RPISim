

% A simple script for testing the simulator.  
% Test the update of the quaternion

scene = 3;

switch scene
    case 1
        % A sphere falling onto a static sphere with a small offset in the
        % +X and +Y directions
        s1 = bodySphere(1,1);
            s1.u = [0;0;0];
            s1.static=1;
            s1.mu = .5;
        s2 = bodySphere(1,1);
            s2.u = [.2;.2;3];
            s2.mu = .5;
            
    case 2
        % Identical to scene 1, except the falling sphere has a rotation of
        % pi/2 about the Z axis
        s1 = bodySphere(1,1);
            s1.u = [0;0;0];
            s1.static=1;
            s1.mu = .5;
        s2 = bodySphere(1,1);
            s2.u = [.2;.2;3];
            s2.quat = quat([0;0;1],pi/2);
            s2.mu = .5;
            
    case 3 % Sphere rolling across top of a static cylinder
        s1 = bodyCylinder(1,5,.3);
            s1. u = [0;0;0];
            s1.quat = quat(rand(3,1),.01);
            s1.static = 1;
            s1.mu = .5;
        s2 = bodySphere(1,1); 
            s2.u = [0;-5;3];
            s2.nu = [0;2;0;0;0;0];
            s2.quat = quat(rand(3,1),pi*rand);
            s2.mu = .5;
end


P = {s1     % Cell vector of all objects.
     s2
     };

% Simulator
sim = Simulation(P,.01);  % sim.DRAW_VECS = 1;
sim.gravityON;
sim.run; 
