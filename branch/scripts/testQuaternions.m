
% Testing some rotations using new quaternion library qt
h = 0.1; 
c1 = mesh_cube();
% Do some operations on the cube
    c1.u = [0;-2;0];
    c1.quat = qt(rand(3,1),pi);
    c1.Fext = zeros(6,1); 
    %c1.faceAlpha = 0.4; 

c2 = mesh_cube();
    c2.u = [0;0;0];
    c2.Fext = zeros(6,1); 
    %c2.faceAlpha = 0.4;
    c2.color = [.2 .3 .4];
    c2.scale(.5);
    c2.quat = qt([0;0;1],pi/2);

c3 = mesh_cube();
    c3.u = [0;2;0];
    %c2.faceAlpha = 0.4;
    c2.Fext = zeros(6,1);
    c2.color = [.6 .2 .2];
    c2.scale(1.4); 
    c2.quat = qt([0;1;0],pi/2);

c1.update_world_position;
c1.draw; hold on;
c2.draw;
c3.draw;

a = 1.5;
axis equal; axis([-a a -3*a 3*a -a a]); grid on;
xlabel('X'); ylabel('Y'); zlabel('Z'); 
rotate3d on; 


% Do some stuff
P = -4*rand(3,1);
pt = sign(rand(1)-.5)*2*rand(3,1); 
c1.applyImpulse( P , c1.u+pt, h);
c2.applyImpulse( P , c2.u+pt, h);
c3.applyImpulse( P , c3.u+pt, h);

drawnow(); view(3); pause(3);
for step = 0:1:400
    c1.kinematicUpdate( h ); 
    c2.kinematicUpdate( h ); 
    c3.kinematicUpdate( h );
    c1.draw(); 
    c2.draw();
    c3.draw(); 
    drawnow(); 
    title(['Step: ' num2str(step)]);
end


