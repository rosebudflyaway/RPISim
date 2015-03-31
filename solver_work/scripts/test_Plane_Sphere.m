
% Creates 3 planes that intersect at (0,0,0), then drops 
% a couple spheres onto them.  

o = [0;0;0];                % Origin
z = [0;0;1];                % Z unit vector
n = [0;1;5]; n = n/norm(n); % A normal vector

% The three planes
p1 = bodyPlane(o,rot(z,-pi/2)*n);
    p1.color = rand(1,3); 
    p1.faceAlpha = 1;
p2 = bodyPlane(o,rot(z, 0)*n);
    p2.color = rand(1,3); 
    p2.faceAlpha = 1;
p3 = bodyPlane(o,rot(z,+pi/2)*n);
    p3.color = rand(1,3); 
    p3.faceAlpha = 1;

% A couple spheres
s1 = bodySphere(1, 0.5);
    s1.setPosition(.5, -.5, 1);
s2 = bodySphere(.5, 0.3);
    s2.setPosition(.5, -.3, 2);

bodies = {p1; p2; p3; s1; s2};
sim = Simulation(bodies, 0.01); 
sim.run(); 
    