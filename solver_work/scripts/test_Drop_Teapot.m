

% test_Drop_Teapot
pot = mesh_read_poly_file('teapot.obj'); 
    pot.u = [0;0;100];
    pot.quat = quat([1;0;0],-pi/2 + .2);
    pot.Fext = [0;0;-9.8;0;0;0];
    pot.color = [.9336 .8528 .6868];
    
pot2 = mesh_read_poly_file('teapot.obj'); 
    pot2.u = [0;0;250];
    pot2.quat = quat([0;1;0],.2);
    pot2.Fext = [0;0;-9.8;0;0;0];
    pot2.color = [.6336 .7528 .9868];
    
cube = mesh_cube;
    cube.setStatic(true);
    cube.scale(200);
    cube.u = [0;0;-100];

bodies = { cube
           pot
           pot2 };
       

sim = Simulation(bodies, 0.01);
sim.SOLVER = 'CDA';
sim.MAX_ITERS = 600;
sim.DRAW = false; 
sim.RECORD_DATA = true;
sim.run;

