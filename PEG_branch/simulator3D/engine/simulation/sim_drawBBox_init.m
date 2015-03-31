%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
%

function sim = sim_drawBBox_init( sim )

    Vbb = zeros(8,3); 
    for b=1:sim.num_bodies
        M = sim.bodies(b).AABB_max;
        m = sim.bodies(b).AABB_min; 
        Vbb(1,:) = [m(1) m(2) m(3)];  % Verts of BBox
        Vbb(2,:) = [m(1) M(2) m(3)];
        Vbb(3,:) = [M(1) M(2) m(3)];
        Vbb(4,:) = [M(1) m(2) m(3)];
        Vbb(5,:) = [m(1) m(2) M(3)];
        Vbb(6,:) = [m(1) M(2) M(3)];
        Vbb(7,:) = [M(1) M(2) M(3)];
        Vbb(8,:) = [M(1) m(2) M(3)];
        Ebb = [ Vbb(1,:); Vbb(2,:)
                Vbb(3,:); Vbb(4,:)
                Vbb(1,:); Vbb(5,:)
                Vbb(6,:); Vbb(7,:)
                Vbb(8,:); Vbb(5,:)
                Vbb(6,:); Vbb(2,:)
                Vbb(3,:); Vbb(7,:)
                Vbb(8,:); Vbb(4,:) ];
        sim.bodies(b).bboxHandle = plot3(Ebb(:,1),Ebb(:,2),Ebb(:,3),'b');
    end

end

