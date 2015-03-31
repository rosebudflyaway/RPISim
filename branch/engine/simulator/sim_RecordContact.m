%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
% sim_RecordContact.m
%
% Stores contact information at the current step.

% con.Bodies     <-- Need to decide on all or just active bodies  

% con.Constraints   <-- Always empty for now -- maybe set up some  alternatively [mu_static(1,1) mu_kinetic(1,1)]
% con.Contacts 
% con.Settings 

% position [(1,3*nb)]
% quaternion [(1,4*nb)]
% velocities [(1,6*nb)]
% contactPairs [(1,3*nc)]    assumes sorted contacts based on pairs
% contactPoints [contactPoint(1,3) contactNormal(1,3) psi(1,1) mu(1,1)]  alternatively [mu_static(1,1) mu_kinetic(1,1)]


function sim_RecordContact( obj )
    step = obj.STEP;
    
    % con.Bodies
    for b=1:length(obj.P)
        
    end
    
    % con.Constraints
    
    % con.Contacts
    for i=1:length(obj.Contacts)
        obj.con(step).Contacts.contact_id = obj.Contacts(i).id; 
        obj.con(step).Contacts.depth = obj.Contacts(i).psi_n(:,1);
        obj.con(step).Contacts.friction = 0.5*obj.P{obj.Contacts(i).body_1} * obj.P{obj.Contacts(i).body_2}; 
        obj.con(step).Contacts.id1 = obj.Contacts(i).body_1;
        obj.con(step).Contacts.id2 = obj.Contacts(i).body_2;
        obj.con(step).Contacts.normal = obj.Contacts(i).normal(:,1); 
        % 3 vectors (in world frame) from center of body to contact point for bodies 1&2.
        obj.con(step).Contacts.point1 = obj.Contacts(i).p1;
        obj.con(step).Contacts.point2 = obj.Contacts(i).p2;
    end
    
    % con.Settings
    obj.con(step).Settings.h = obj.h; 
end

