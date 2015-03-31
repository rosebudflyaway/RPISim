%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RPI-MATLAB-Simulator
% http://code.google.com/p/rpi-matlab-simulator/
% PEGdynamics.m
%
% Formulate CDA as an MCP and return updated velocities
%

% PEGdynamics assumes a particular set of variables exists, namely
%   sim.contacts    % A struct array of Contact
%   sim.Xcontacts   % A struct array of crossContact
%
function newNU = mPEGdynamics( sim )

    %% Useful vars
    nb = sim.num_activeBodies;    % Number of bodies with contacts
    nc = length(sim.contacts);    % Number of contacts
    nic = length(sim.Iconstraints);
    nxc = length(sim.Xconstraints); 
    nixc = nic+nxc;
    nd = sim.num_fricdirs;        % Number of directions in discrete friction "cone"
    
    ns = length(sim.contacts);
    for i=1:length(sim.contacts)
       ns = ns + length(sim.contacts(i).psi_n)-1; 
    end


    %% Init submatrices
    M = zeros(3*nb,3*nb);
    Gn = zeros(3*nb,nc);
    NU = zeros(3*nb,1);   % Velocity, including angular
    FX = zeros(3*nb,1);   % External force (not impulse!)
    PSI = zeros(nc,1);    % Gap distance per contact, psi_n
    sim.num_subContacts = 0;


    %% Calculate submatrices
    
    % M, NU, and FX
    for i=1:length(sim.bodies)
        B = sim.bodies(i);
        if B.active && B.dynamic
            cID = B.bodyContactID;
            M(3*cID-2:3*cID,3*cID-2:3*cID) = diag([B.mass B.mass B.J]);
            NU(3*cID-2:3*cID) = B.nu;                  % NU
            FX(3*cID-2:3*cID) = B.Fext;                % FX
        end
    end

    %% Formulate additional submatrices
    % mPEG matrices are combinations of contact wrenches.  Construct Gn for
    % body1 and body2 for each contact
    Gn1 = zeros(3,nc);
    Gn2 = zeros(3,nc);
    Gd = zeros(3*nb,nc);    
    Gc = zeros(3*nb, nixc);
    Ge = zeros(3*nb, nixc);
    Gp = zeros(3*nb, nc);
    psi_c = zeros(nixc,1); 
    psi_e = zeros(nixc,1);  
    psi_d = zeros(nc,1); 
    psi_p = zeros(nc,1); 
    Ed = eye(nc);
    Ec = eye(nixc);
    Ee = -eye(nixc);
    Eec = eye(nixc);
    Epd = eye(nc);
    Epc = zeros(nc,nixc);
    
    %% Iterate over contacts
    for i=1:nc
        C = sim.contacts(i); 
        n = C.normal;
        B1 = sim.bodies(C.body1_id);
        B2 = sim.bodies(C.body2_id); 
        r1 = C.p1 - B1.pos'; 
        r2 = C.p1 + C.psi_n*n - B2.pos'; 
        Gn1(:,i) = [-n'; cross2d(r1,-n)];
        Gn2(:,i) = [ n'; cross2d(r2, n)];
        
        % Begin to populate PSI
        PSI(i) = C.psi_n; 
        
        % While looping over the contacts, let's also form Gd
        Gd(3*C.body1_id-2:3*C.body1_id, i) = Gn1(:,i);
        Gd(3*C.body2_id-2:3*C.body2_id, i) = Gn2(:,i);
        psi_d(i) = PSI(i);
    end

    
    %% Iterate over inter-contact constraints
    for i=1:size(sim.Iconstraints,2)
        I = sim.Iconstraints(i); 
        b1id = sim.bodies(sim.contacts(I.c1).body1_id).bodyContactID;
        b2id = sim.bodies(sim.contacts(I.c2).body2_id).bodyContactID; 
        
        % Gc
        Gc(3*b1id-2:3*b1id, i) = Gn1(:,I.c2) - Gn1(:,I.c1);
        Gc(3*b2id-2:3*b2id, i) = Gn2(:,I.c2) - Gn2(:,I.c1);
        psi_c(i) =  PSI(I.c2) - PSI(I.c1); 
        
        % Ge
        Ge(3*b1id-2:3*b1id, i) = Gn1(:,I.c2);
        Ge(3*b2id-2:3*b2id, i) = Gn2(:,I.c2);
        psi_e(i) = PSI(I.c2);
        
        % Gp (ns entries per inter-contact constraint, assumed 2 for now)
        Gp(3*b1id-2:3*b1id, 2*i-1) = Gn1(:,I.c2);
        Gp(3*b2id-2:3*b2id, 2*i-1) = Gn2(:,I.c2);
        psi_p(2*i-1) = PSI(I.c2);
        
        Gp(3*b1id-2:3*b1id, 2*i) = Gn1(:,I.c2);
        Gp(3*b2id-2:3*b2id, 2*i) = Gn2(:,I.c2);
        psi_p(2*i) = PSI(I.c2);
        
        % Epc
        Epc(2*i-1:2*i,i) = 1;
    end
    clear I;
    
    %% Iterate over extra-contact constraints
    for j = 1:length(sim.Xconstraints)
        i = j+length(sim.Iconstraints); 
        X = sim.Xconstraints(j); 
        
        % There are two body ids PER contact, 
        
        % Second contact (two + Gn)
        b1id = sim.bodies(sim.contacts(X.c2).body1_id).bodyContactID;
        b2id = sim.bodies(sim.contacts(X.c2).body2_id).bodyContactID; 
        Gc(3*b1id-2:3*b1id, i) = Gn1(:,X.c2); % - Gn1(:,X.c1);
        Gc(3*b2id-2:3*b2id, i) = Gn2(:,X.c2); % - Gn2(:,X.c1);
        
        % First contact (two - Gn)
        b1id = sim.bodies(sim.contacts(X.c1).body1_id).bodyContactID;
        b2id = sim.bodies(sim.contacts(X.c1).body2_id).bodyContactID; 
        Gc(3*b1id-2:3*b1id, i) = Gc(3*b1id-2:3*b1id, i) - Gn2(:,X.c1);  %% TODO TODO TODO
        Gc(3*b2id-2:3*b2id, i) = Gc(3*b2id-2:3*b2id, i) - Gn1(:,X.c1);  % Gn1 and Gn2 don't correspond to b1id and b2id!!!
        
        % Psi_c
        psi_c(i) =  PSI(X.c2) - PSI(X.c1); 
        
        b1id = sim.bodies(sim.contacts(X.c2).body1_id).bodyContactID;
        b2id = sim.bodies(sim.contacts(X.c2).body2_id).bodyContactID; 
        % Ge
        Ge(3*b1id-2:3*b1id, i) = Gn1(:,X.c2);
        Ge(3*b2id-2:3*b2id, i) = Gn2(:,X.c2);
        psi_e(i) = PSI(X.c2);
    end

    %% Construct A and finish b
    % Currently, no friction constraints
    A = [ -M    zeros(3*nb,2*nixc+nc)           Gd
           Gc'  Ec   zeros(nixc,2*nc+nixc)
           Gd'  zeros(nc,nixc)  Ed    zeros(nc,nixc+nc)
           Ge'  Eec  zeros(nixc,nc)   Ee        zeros(nixc,nc)
           Gp'  Epc  Epd  zeros(nc,nixc+nc)   ];

    % Construct b
    activeBody = 1;
    for i=1:length(sim.bodies)
        if sim.bodies(i).active
           low = 3*activeBody-2; high = 3*activeBody;
           b(low:high) = M(low:high,low:high) * sim.bodies(i).nu + sim.h*sim.bodies(i).Fext;
           activeBody = activeBody + 1;
        end
    end
    
    h = sim.h; 
    b = [ b'
          psi_c / h
          psi_d / h
          psi_e / h
          psi_p / h ]; 
    
    % Solve the MCP  
    problem_size = size(A,1);
    z0 = zeros(problem_size,1);       
    big=10^20;    
    u = big*ones(problem_size,1);
    l = zeros(size(u));
    l(1:3*nb) = -big;

    newNU = pathlcp(A,b,l,u,z0);

end




