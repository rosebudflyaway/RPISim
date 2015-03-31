
% Given bodies A and B in 2D, returns the indices of the two nearest 
% vertices from each body.  
function [Dmin, vai, vbi] = getNearestVerts( A, B )

    function sqD = sqDist(V1,V2)
       sqD = sum((V1-V2).^2); 
    end

    % Initialize
    vai = 1;
    vbi = 1;
    va = A.verts_world(vai,:);
    vb = B.verts_world(vbi,:);
    Dmin = sqDist(va,vb); 
    
    vertMoved = true;
    while vertMoved
       vertMoved = false;  
       
       if vai == A.num_verts
          vaip1 = 1;
       else
          vaip1 = vai+1;
       end
       vap1 = A.verts_world(vaip1,:); 
       if vai == 1
          vaim1 = A.num_verts; 
       else
          vaim1 = vai-1; 
       end
       vam1 = A.verts_world(vaim1,:);  
       
       if vbi == B.num_verts
          vbip1 = 1;
       else
          vbip1 = vbi+1;
       end
       vbp1 = B.verts_world(vbip1,:); 
       if vbi == 1
          vbim1 = B.num_verts; 
       else
          vbim1 = vbi-1; 
       end
       vbm1 = B.verts_world(vbim1,:);  
       
       
       %% test va-1 on vb-1, vb, and vb+1
       d = sqDist(vam1, vbm1); 
       if d < Dmin, 
           Dmin = d; 
           vai = vaim1; va = vam1; 
           vbi = vbim1; vb = vbm1; 
           vertMoved = true; 
       end
       
       d = sqDist(vam1, vb); 
       if d < Dmin, 
           Dmin = d; 
           vai = vaim1; va = vam1; 
           vertMoved = true; 
       end
       
       d = sqDist(vam1, vbp1); 
       if d < Dmin, 
           Dmin = d; 
           vai = vaim1; va = vam1; 
           vbi = vbip1; vb = vbp1; 
           vertMoved = true; 
       end
       
       
       %% test va+1 on vb-1, vb, and vb+1
       d = sqDist(vap1, vbm1); 
       if d < Dmin, 
           Dmin = d; 
           vai = vaip1; va = vap1; 
           vbi = vbim1; vb = vbm1; 
           vertMoved = true; 
       end
       
       d = sqDist(vap1, vb); 
       if d < Dmin, 
           Dmin = d; 
           vai = vaip1; va = vap1; 
           vertMoved = true; 
       end
       
       d = sqDist(vap1, vbp1); 
       if d < Dmin, 
           Dmin = d; 
           vai = vaip1; va = vap1; 
           vbi = vbip1; vb = vbp1; 
           vertMoved = true; 
       end
       
       
       %% test va on vb-1 and vb+1
       d = sqDist(va, vbm1); 
       if d < Dmin, 
           Dmin = d; 
           vbi = vbim1; vb = vbm1; 
           vertMoved = true; 
       end
       
       d = sqDist(va, vbp1); 
       if d < Dmin, 
           Dmin = d; 
           vbi = vbip1; vb = vbp1; 
           vertMoved = true; 
       end
       
       %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
       %% test vb-1 on va-1, va, and va+1
       d = sqDist(vbm1, vam1); 
       if d < Dmin, 
           Dmin = d; 
           vai = vaim1; va = vam1; 
           vbi = vbim1; vb = vbm1; 
           vertMoved = true; 
       end
       
       d = sqDist(vbm1, va); 
       if d < Dmin, 
           Dmin = d; 
           vbi = vbim1; vb = vbm1; 
           vertMoved = true; 
       end
       
       d = sqDist(vbm1, vap1); 
       if d < Dmin, 
           Dmin = d; 
           vai = vaip1; va = vap1; 
           vbi = vbim1; vb = vbm1; 
           vertMoved = true; 
       end
       
       %%
       d = sqDist(vbp1, va); 
       if d < Dmin, 
           Dmin = d; 
           vai = vaim1; va = vam1; 
           vbi = vbip1; vb = vbp1; 
           vertMoved = true; 
       end
       
       d = sqDist(vbp1, va); 
       if d < Dmin, 
           Dmin = d; 
           vbi = vbip1; vb = vbp1; 
           vertMoved = true; 
       end
       
       d = sqDist(vbm1, vap1); 
       if d < Dmin, 
           Dmin = d; 
           vai = vaip1; va = vap1; 
           vbi = vbip1; vb = vbp1; 
           vertMoved = true; 
       end
       
       %% 
       d = sqDist(vb, vam1); 
       if d < Dmin, 
           Dmin = d; 
           vai = vaim1; va = vam1; 
           vertMoved = true; 
       end

       d = sqDist(vbm1, vap1); 
       if d < Dmin, 
           Dmin = d; 
           vai = vaip1; va = vap1; 
           vertMoved = true; 
       end

       
    end

end










