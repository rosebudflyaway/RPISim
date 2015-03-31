

function numberVerts( sim )

    for bid = 1:sim.num_bodies
       B = sim.bodies(bid);
       center = mean(B.verts_world);
       for vid = 1:B.num_verts
          vb = B.verts_world(vid,:);
          vb = vb - 0.07*(vb - center);
          text(vb(1),vb(2),num2str(vid)); 
       end
    end

end

