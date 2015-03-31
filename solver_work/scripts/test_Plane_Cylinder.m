

P = bodyPlane([0;0;0],[0;0;1]);
C = bodyCylinder(1.5, 0.3, 1.2);
    C.setPosition(1,1,2);
    C.quat = quat([0;1;0] , pi*1.3);
    
sim = Simulation({P;C},0.01);
sim.MAX_ITERS = 500;
%sim.disableGUI();
%sim.setRecord(true);  
sim.run(); 




