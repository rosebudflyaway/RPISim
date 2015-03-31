

% Given bodies A and B, returns the indices va and vb of the respective
% world vertices nearest the other body.  
% 
%   OUTPUT:
%           sd  - Squared distance between features
%           va  - Index of the vertex in A that is nearest body B
%           vb  - Index of the vertex in B that is nearest body A


function [ sd, va, vb ] = getNearestFeatures2d( A, B )

    
    function vip1 = prevVertex( bdy, vi )
        if vi == 1
            vip1 = bdy.num_verts;
        else
            vip1 = vi - 1;
        end
    end
    function vip1 = nextVertex( bdy, vi )
        if vi == bdy.num_verts
            vip1 = 1;
        else
            vip1 = vi + 1;
        end
    end

    %% Determine va
    va = 1;         % TODO: use spacial-temporal coherence to init
    vb = 1;
    sd = sqDist_point_segment2d( B.verts_world(vb,:), ... 
                                 B.verts_world(nextVertex(B,vb),:), ... 
                                 A.verts_world(va,:) ); 
                             
    feature_change = true; 
                             
    % Loop over pairs of vertex va and edge v_b+1 - v_b
    while feature_change
        
        % Case 1. va,vb    (finished)
        % Case 2. va,vb-1
        % Case 3. va,vb+1
        
        % Case 4. va-1,vb
        % Case 5. va-1,vb-1
        % Case 6. va-1,vb+1
        
        % Case 7. va+1,vb
        % Case 8. va+1,vb-1
        % Case 9. va+1,vb+1
        
        vam1 = prevVertex(A,va);
        vap1 = nextVertex(A,va);
        vbm1 = prevVertex(B,vb);
        vbp1 = nextVertex(B,vb); 
        vbp2 = nextVertex(B,vbp1);
        
        sd2 = sqDist_point_segment2d( B.verts_world(vbm1,:), ...
                                      B.verts_world(vb,:), ...
                                      A.verts_world(va,:) ); 
        sd3 = sqDist_point_segment2d( B.verts_world(vbp1,:), ...
                                      B.verts_world(vbp2,:), ...
                                      A.verts_world(va,:) ); 
                                  
        sd4 = sqDist_point_segment2d( B.verts_world(vb,:), ...
                                      B.verts_world(vbp1,:), ...
                                      A.verts_world(vam1,:) ); 
        sd5 = sqDist_point_segment2d( B.verts_world(vbm1,:), ...
                                      B.verts_world(vb,:), ...
                                      A.verts_world(vam1,:) ); 
        sd6 = sqDist_point_segment2d( B.verts_world(vbp1,:), ...
                                      B.verts_world(vbp2,:), ...
                                      A.verts_world(vam1,:) ); 
                                  
        sd7 = sqDist_point_segment2d( B.verts_world(vb,:), ...
                                      B.verts_world(vbp1,:), ...
                                      A.verts_world(vap1,:) ); 
        sd8 = sqDist_point_segment2d( B.verts_world(vbm1,:), ...
                                      B.verts_world(vb,:), ...
                                      A.verts_world(vap1,:) ); 
        sd9 = sqDist_point_segment2d( B.verts_world(vbp1,:), ...
                                      B.verts_world(vbp2,:), ...
                                      A.verts_world(vap1,:) );   
                                  
        [sdnew sdi] = min([sd sd2 sd3 sd4 sd5 sd6 sd7 sd8 sd9]);                       
                                  
        sd = sdnew;  
        switch sdi
            case 1
                feature_change = false;
            case 2
                vb = vbm1;
            case 3
                vb = vbp1;
            case 4
                va = vam1;
            case 5
                va = vam1;
                vb = vbm1;
            case 6
                va = vam1;
                vb = vbp1;
            case 7
                va = vap1;
            case 8
                va = vap1;
                vb = vbm1;
            case 9
                va = vap1;
                vb = vbp1;
        end
                                  
    end
    

end

















