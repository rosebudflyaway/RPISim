
% INPUT:
%       sim     - Simulator object
%       start_index - Index to start searching 
%       b1id    - ID of body1
%       b2id    - ID of body2
%       f1id    - ID of feature on body1
%       f2id    - ID of feature on body2
% 
% OUTPUT:
%       cid     - The index of the contact with this information
%       

function cid = get_contactID( sim, start_index, b1id, b2id, f1id, f2id )

    C = sim.contacts;
    cid = start_index;
    while cid <= length(C)
       c = C(cid);
       if c.body1_id == b1id && c.body2_id == b2id && c.f1id == f1id && c.f2id == f2id
           return;
       end
       cid = cid+1;
    end
    
    if cid > length(C);
        cid = 0;
    end

end

