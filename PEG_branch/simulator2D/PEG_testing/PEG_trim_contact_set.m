

% Given a set of contacts between A and B, assuming all contacts are
% between a single vertex va of A with adjacent edges of B, returns the
% same set of contacts but ordered by their adjacency.  For example, given
% edges of B in terms of tail vertices of B
%      vb11----vb12----vb1-----vb2----vb3
% where there are four contacts in C:
%   c1 = C(va, vb1)
%   c2 = C(va, vb2)
%   c3 = C(va, vb11)
%   c4 = C(va, vb12)
% this function reorders the contacts to reflect their geometry, not their
% vertex indices in B.  The output for this example would be
%   c1 = C(va, vb11)
%   c2 = C(va, vb12)
%   c3 = C(va, vb1)
%   c4 = C(va, vb2)
%
% There is an assumption made here that there is at most one "split" in the
% contact set.  
% 
% Further, this function removes contacts that are more than one contact
% beyond applicability.  

function C = PEG_trim_contact_set( C, eps_theta ) 

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


    %% First ensure that contact set is in proper order of f2b
    % We assume that there is at most one "split" in the contact set that
    % needs to be corrected, so we search for a jump in vertex index.
    ci = 1;
    while ci < length(C)
        if C(ci).f2id+1 ~= C(ci+1).f2id   
           C = [C(ci:end) C(1:ci-1)];
           break;
        end
        ci = ci+1;
    end
    
    
    %% Second trim the contact set to exclude contacts beyond one contact of applicability
    c_start = 1;
    if ~(C(c_start).applicability > eps_theta)
        while c_start < length(C)-1 % We use -1 because we wish to keep AT LEAST 2 contacts
            if C(c_start+1).applicability > eps_theta
                break;
            end
            c_start = c_start + 1; 
        end
    end
    c_end = length(C);
    if ~(C(c_end).applicability > eps_theta)
        while c_end > 2 % We use 2 because we wish to keep AT LEAST 2 contacts
            if C(c_end-1).applicability > eps_theta
                break;
            end
            c_end = c_end - 1; 
        end
    end
    
    %disp([num2str(length(C)) ' --> '  num2str(max([0 c_end-c_start+1]))]);
    C = C(c_start:c_end);
    
    
    % In 2D, let's keep only the 2 nearest contacts.  
    % TODO
    

end






