

%
% INPUT
%       A       - Body
%       va_min  - First index of feature set on A
%       va_max  - Last index of feature set on A
%       B       - Body
%       vb_min  - First index of feature set on B
%       vb_max  - Last index of feature set on B
% 
% OUTPUT
%       C       - A set of contacts 
%       U       - A set of unilateral constraints
%       I       - A set of inter-contact constraints
%       X       - A set of cross-contact constraints
%


function [C, U, I, X] = PEG_narrow_phase( A, va_min, va_max, B, vb_min, vb_max, eps_ve )

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

    %% Determine the full set of vertex-edge contacts 
    
    % A onto B
    va = va_min;
    while true
       vb = vb_min;
       while true
          
          vb = nextVertex(B, vb);
          if vb == vb_max, break; end
       end
       va = nextVertex(A, va); 
       if va == va_max, break; end
    end
    
    

end














