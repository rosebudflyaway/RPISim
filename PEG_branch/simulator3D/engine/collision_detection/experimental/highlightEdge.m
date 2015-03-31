


function highlightEdge( body, edge )

   E = body.edges(edge,:); 
   v1 = body.verts_world(E(1),:);
   v2 = body.verts_world(E(2),:); 
   
   % Plot edge
   plot3([v1(1) v2(1)],[v1(2) v2(2)],[v1(3) v2(3)],'r--');
   
   % Plot T-vectors
   t1 = body.tvecs(edge,1:3)/2;
   t2 = body.tvecs(edge,4:6)/2; 
   p = v1+(v2-v1)/2;
   plot3([p(1) p(1)+t1(1)],[p(2) p(2)+t1(2)],[p(3) p(3)+t1(3)],'yellow');
   plot3([p(1) p(1)+t2(1)],[p(2) p(2)+t2(2)],[p(3) p(3)+t2(3)],'yellow');
   
   % Plot face normals
   pf1 = p + t1/2;
   pf2 = p + t2/2;
   nf1 = body.face_norms(E(3),:)/4;
   nf2 = body.face_norms(E(4),:)/4;
   plot3([pf1(1) pf1(1)+nf1(1)],[pf1(2) pf1(2)+nf1(2)],[pf1(3) pf1(3)+nf1(3)],'cyan');
   plot3([pf2(1) pf2(1)+nf2(1)],[pf2(2) pf2(2)+nf2(2)],[pf2(3) pf2(3)+nf2(3)],'cyan');
   
end



