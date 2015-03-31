

% INPUT
%       A       - A Body object.
%       B       - A body object.
%       eps_ve  - Epsilon for vertex-edge distance  
%       
% OUTPUT
%       vb_min  - The first vertex in Vb
%       vb_max  - The last vertex in Vb
%
%   It is important to note that:
%       1. A and B are convex polyhedra
%       2. The sets Va and Vb will each be connected, so we can keep track
%       of the first and last index of each set.  
% 

function [ vb_min, vb_max ] = getActiveEdges2d( A, B, eps_ve )

    eps_appl = -0.25;  % Applicability relaxation 
    eps_ve = eps_ve*eps_ve;  

    %% Utility functions
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



    %% Determine initial va and vb
    [ sd, va_min, vb_min ] = getNearestFeatures2d( A, B );  
    va_max = va_min;
    vb_max = vb_min;
    
    % 
    if sd > eps_ve
       vb_min = [];
       vb_max = [];
       return; 
    end
    
    
    %% Expand Vb in A's negative direction
    va = va_min; 
    vaUpdate = true; 
    while vaUpdate
        
        vaUpdate = false; 
        
        % Expand in B's negative direction
        %vb = vb_min;  
        vbUpdate = true;
        while vbUpdate
            vbUpdate = false; 
            vbm1 = prevVertex(B,vb_min); 
            sqDist = sqDist_point_segment2d( B.verts_world(vbm1,:), ...
                                             B.verts_world(vb_min,:), ...
                                             A.verts_world(va,:) );  
            if sqDist <= eps_ve
                vaUpdate = true; 
                vb_min = vbm1;  
                if APPL_vertex_edge(A,va,B,vbm1) > eps_appl
                    vbUpdate = true; 
                end
            end
        end
        
        % Expand in B's positive direction
        %vb = vb_max;  
        vbUpdate = true;
        while vbUpdate
            vbUpdate = false; 
            vbp1 = nextVertex(B,vb_max); 
            sqDist = sqDist_point_segment2d( B.verts_world(vb_max,:), ...
                                             B.verts_world(vbp1,:), ...
                                             A.verts_world(va,:) );  
            if sqDist <= eps_ve
                vaUpdate = true; 
                if APPL_vertex_edge(A,va,B,vb_max) > eps_appl
                    vbUpdate = true; 
                end
                vb_max = vbp1;  
            end
        end
        
        % Keep moving va
        va = prevVertex(A,va); 
    end
    
    
    %% Expand Vb in A's positive direction
    %va = va_max;        
    va = nextVertex(A,va_max);  
    vaUpdate = true; 
    while vaUpdate
        
        vaUpdate = false; 
        
        % Expand in B's negative direction
        %vb = vb_min;  
        vbUpdate = true;
        while vbUpdate
            vbUpdate = false; 
            vbm1 = prevVertex(B,vb_min); 
            sqDist = sqDist_point_segment2d( B.verts_world(vbm1,:), ...
                                             B.verts_world(vb_min,:), ...
                                             A.verts_world(va,:) );  
            if sqDist <= eps_ve
                vaUpdate = true; 
                vb_min = vbm1;  
                if APPL_vertex_edge(A,va,B,vbm1) > eps_appl
                    vbUpdate = true; 
                end
            end
        end
        
        % Expand in B's positive direction
        %vb = vb_max;  
        vbUpdate = true;
        while vbUpdate
            vbUpdate = false; 
            vbp1 = nextVertex(B,vb_max); 
            sqDist = sqDist_point_segment2d( B.verts_world(vbp1,:), ...
                                             B.verts_world(nextVertex(B,vbp1),:), ...
                                             A.verts_world(va,:) );  
            if sqDist <= eps_ve
                vaUpdate = true;  
                if APPL_vertex_edge(A,va,B,vb_max) > eps_appl
                    vbUpdate = true; 
                end
                vb_max = vbp1; 
            end
        end
        
        % Keep moving va
        va = nextVertex(A,va); 
    end

    
    %% DEBUG
    vb = vb_min;
    V1 = B.verts_world(vb,:);
    plot(V1(1),V1(2),'g*');
    while vb ~= vb_max 
        vbp1 = nextVertex(B,vb);
        V1 = B.verts_world(vb,:);
        V2 = B.verts_world(vbp1,:);
        plot([V1(1) V2(1)],[V1(2) V2(2)],'g','linewidth',2);  

        vb = vbp1;
        plot(V1(1),V1(2),'g*');
    end
    
end










