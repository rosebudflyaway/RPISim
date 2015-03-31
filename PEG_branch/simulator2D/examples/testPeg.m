

d = 0.01;
A = Body([-0.5;-0.5;1.5;1.5],[2;1+d;1+d;2]);
    A.color = [1 0 0];

B = Body([1;1;2],[1;-0.5;1]);


sim = Simulator();
sim = sim_addBody( sim, [A B] );
%sim.MAX_STEP = 0; 

sim = sim_run( sim );



[Dmin, vai, vbi] = getNearestVerts( A, B ); 

plot(A.verts_world(vai,1),A.verts_world(vai,2),'bx');
plot(B.verts_world(vbi,1),B.verts_world(vbi,2),'bo');



